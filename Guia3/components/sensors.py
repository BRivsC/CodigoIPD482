import random
import numpy as np
import base64

    # --- IMUSensor -----------------------------------------------------------
class IMUSensor:
    def __init__(self, sim, gyro_handle, acc_handle,
                 gyro_noise_deg=0.5, acc_noise=0.2, bias_deg=0.001):
        self.sim = sim
        self.gyro = gyro_handle
        self.acc  = acc_handle
        self.gyro_sigma = np.deg2rad(gyro_noise_deg)
        self.acc_sigma  = acc_noise
        self.bias = np.deg2rad(bias_deg) * np.random.randn()

    def read(self):
        # ----- gyro Z -----
        gyro_val = self.sim.getFloatSignal(self.gyro)
        if gyro_val is None:                       # <-- si no hay señal → 0
            gyro_val = 0.0
        gyro_z = gyro_val + self.gyro_sigma*np.random.randn() + self.bias

        # ----- accel X -----
        acc_val = self.sim.getFloatSignal(self.acc)
        if acc_val is None:
            acc_val = 0.0
        acc_x = acc_val + self.acc_sigma*np.random.randn()

        return gyro_z, acc_x

# --- GPSSensor -----------------------------------------------------------
class GPSSensor:
    def __init__(self, sim, handle,
                 sigma_xy=1.0, sigma_yaw = 0.01, freq_hz=5.0,
                 robot_handle=None, trailer_handle=None):
        self.sim   = sim
        self.h     = handle            # GPS principal
        self.robot_handle   = robot_handle
        self.trailer_handle = trailer_handle
        self.sigma  = sigma_xy
        self.sigma_yaw = np.deg2rad(sigma_yaw)  # ruido en orientación
        self.period = 1.0 / freq_hz
        self._next_t = 0.0

    def read_if_ready(self):
        """
        Devuelve la posición actual (x,y,z) con ruido.
        Si no hay señal, devuelve None.
        """
        t = self.sim.getSimulationTime()
        if t < self._next_t:
            return None
        self._next_t += self.period
        pos = self.sim.getObjectPosition(self.h, -1)  # world
        noisy = np.array(pos[:2]) + self.sigma*np.random.randn(2)
        return noisy

    def get_position(self):
        """Devuelve la posición actual (x,y,z) con ruido."""
        pos = self.sim.getObjectPosition(self.h, -1)
        noisy = np.array(pos) + self.sigma*np.random.randn(3)
        return noisy  # [x, y, z]

    def get_real_position(self):
        """Alias del anterior para compatibilidad."""
        return self.get_position()

    
    def get_trailer_position(self):
        """Posición del tráiler o [0,0,0] si no hay handle."""
        if self.trailer_handle is not None:
            return self.sim.getObjectPosition(self.trailer_handle, -1)
        return [0, 0, 0]
        
    def get_orientation(self):
        """Orientación del robot (roll,pitch,yaw)."""
        if self.robot_handle is not None:
            roll, pitch, yaw = self.sim.getObjectOrientation(self.robot_handle, -1)
            yaw_noisy = yaw + self.sigma_yaw * np.random.randn() 
            return [roll, pitch, yaw_noisy]
        return [0, 0, 0]

        
    def get_trailer_orientation(self):
        """Orientación del tráiler o ceros si no hay handle."""
        if self.trailer_handle is not None:
            return self.sim.getObjectOrientation(self.trailer_handle, -1)
        return [0, 0, 0]

class HokuyoSensor:
    def __init__(self, sim, handle,
                 ranges_sig='fastHokuyo_scanRangesB64',
                 angles_sig='fastHokuyo_scanAnglesB64',
                 max_range=5.0,
                 sensor_offset_xy=(0.06, 0.00),       # dx, dy  [m]
                 sensor_yaw_offset_deg=90.0):         # giro fijo [deg]
        self.sim        = sim
        self.h          = handle
        self.ranges_sig = ranges_sig
        self.angles_sig = angles_sig
        self.max_range  = max_range
        self.off_x, self.off_y = sensor_offset_xy
        self.off_yaw    = np.deg2rad(sensor_yaw_offset_deg)

        # --- live plot handles (se inicializan al primer uso) ---
        self._fig = None  # figura de matplotlib
        self._ax  = None  # axes
        self._sc  = None  # scatter plot

        # --- plot en coordenadas mundo ---
        self._fig_w = None
        self._ax_w  = None
        self._sc_w  = None

        # --- mapa acumulado ---
        self._map_cloud = np.empty((0, 2), dtype=np.float32)
        self._fig_map = None
        self._ax_map  = None
        self._sc_map  = None

        # --- multi-window plot unified in subplots ---
        self._fig_all = None      # figura compartida
        self._ax_local = None
        self._ax_world = None  # deprecated (no se usa)
        self._ax_map_ax = None
        self._sc_local = None
        self._sc_world = None  # deprecated
        self._sc_map_shared = None

    def get_landmarks(self):
        # Obtiene la nube original con: sim.readProximitySensor / signal stream
        pts = self.sim.getStringSignal(self.h)  # <-- ajusta a tu método real
        if not pts:      # no hay lecturas
            return []
        pts = np.frombuffer(pts, dtype=np.float32).reshape(-1,3)[:,:2]  # (x,y)
        # clustering SIMPLE: k-means++ centrado en puntos más lejanos a (0,0)
        if pts.shape[0] == 0: return []
        idx = np.argsort(-np.linalg.norm(pts, axis=1))[:self.max_feats]
        feats = []
        for j,i in enumerate(idx):
            x,y = pts[i]
            r   = np.sqrt(x*x+y*y) + self.r_sigma*np.random.randn()
            beta= np.arctan2(y,x) + self.b_sigma*np.random.randn()
            feats.append( (r, beta, j) )   # id = j
        return feats
    
    def get_xy_world(self):
        """
        Devuelve la nube de puntos en el mundo.
        Si no hay señal, devuelve un array vacío.
        """
        xy_b64 = self.sim.getStringSignal('fastHokuyo_scanXYB64')
        if not xy_b64:
            return np.empty((0, 2))
        flat = np.frombuffer(base64.b64decode(xy_b64), dtype=np.float32)

        return flat.reshape(-1, 2)  # (N,2)

    def world_to_vehicle(self, pts_world, veh_pose):
        """
        Convierte una nube 2-D de coordenadas mundo al frame del vehículo.

        Parameters
        ----------
        pts_world : ndarray, shape (N, 2)
            Puntos [Xw, Yw] que obtienes de get_xy_world().
        veh_pose : iterable, length 7
            Pose del vehículo tal y como la devuelve sim.getObjectPose():
            [x, y, z, qx, qy, qz, qw]  (quaternion world→vehicle).

        Returns
        -------
        pts_local : ndarray, shape (N, 2)
            Los mismos puntos expresados en el frame del vehículo.
            (0,0) es ahora el sensor; +X apunta hacia adelante;
            +Y a la izquierda (convención CoppeliaSim).
        """
        if pts_world.size == 0:
            return pts_world.copy()          # vacío → vacío

        # --- 1. extraer traslación (x, y) y yaw θ ---
        xw, yw = veh_pose[0], veh_pose[1]

        if len(veh_pose) >= 7:
            # Pose completa con quaternion (x,y,z,qx,qy,qz,qw)
            qx, qy, qz, qw = veh_pose[3:]

            # Yaw (rotación alrededor de Z).  Fórmula estándar de "yaw-pitch-roll"
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

        elif len(veh_pose) == 3:
            # pose simplificada (x,y,yaw)
            yaw = veh_pose[2]
        else:
            raise ValueError("veh_pose debe ser longitud 7 (pose completa) o 3 (x,y,yaw)")

        # --- 2. quitar offset del sensor (posición real del láser) ---
        xs = xw + self.off_x * np.cos(yaw) - self.off_y * np.sin(yaw)
        ys = yw + self.off_x * np.sin(yaw) + self.off_y * np.cos(yaw)
        pts_rel = pts_world - np.array([xs, ys])     # (N,2)

        # --- 3. rotación inversa: girar −(yaw + off_yaw) ---
        c, s = np.cos(-(yaw + self.off_yaw)), np.sin(-(yaw + self.off_yaw))
        R = np.array([[c, -s],
                    [s,  c]])                      # 2×2
        pts_local = pts_rel @ R.T                    # (N,2)

        return pts_local

        #     """
        #     pts_world : (N,3) o (N,2)  puntos en mundo
        #     veh_pose  : [x,y,z, qx,qy,qz,qw]  pose del vehículo
        #     Devuelve los puntos expresados en el frame del vehículo.
        #     """

    # ------------------------------------------------------------------
    # Visualización en tiempo real
    # ------------------------------------------------------------------
    def plot_local(self, veh_pose, pause: float = 0.001):
        from components.visualization import hokuyo_plot_local
        hokuyo_plot_local(self, veh_pose, pause)

    def plot_world(self, pause: float = 0.001):
        # plot_world was deprecated in favour of plot_map, mantenemos alias.
        from components.visualization import hokuyo_plot_map
        hokuyo_plot_map(self, pause)

    def plot_map(self, pause: float = 0.001, subsample: int = 1):
        from components.visualization import hokuyo_plot_map
        hokuyo_plot_map(self, pause, subsample)

    # ------------------------------------------------------------------
    def _ensure_multi_fig(self):
        # Centralizado en components.visualization
        from components.visualization import _ensure_hokuyo_fig
        _ensure_hokuyo_fig(self)

    def save_map_pdf(self, filename: str = 'map_odom.pdf'):
        # Delegar en módulo visualization para mantener separación de capas
        from components.visualization import hokuyo_save_map_pdf
        hokuyo_save_map_pdf(self, filename)

class OdometrySensor:
    """
    Estima la pose del robot integrando los encoders de rueda y construye
    un mapa 2-D en tiempo real acumulando nubes LIDAR. Sirve para ilustrar
    la típica distorsión en "banana" cuando solo se usa odometría.
    """
    def __init__(self, sim, left_handle, right_handle,
                 *, init_x: float = 0.0, init_y: float = 0.0, init_yaw: float = 0.0,
                 wheel_radius=0.0975, wheel_base=0.381,
                 bias_right: float = 1.001,
                 enable_live_plot: bool = True,
                 plot_xlim=(-12, 20), plot_ylim=(-12, 20)):
        import numpy as np  # ensure local namespace
        self.sim = sim
        self.hL = left_handle
        self.hR = right_handle
        self.R = wheel_radius
        self.B = wheel_base
        self.bias_R = bias_right

        # encoder prev readings (rad)
        self.prev_L = self.sim.getJointPosition(self.hL)
        self.prev_R = self.sim.getJointPosition(self.hR)

        # pose estimate
        self.x = float(init_x)
        self.y = float(init_y)
        self.th = float(init_yaw)

        # mapa acumulado
        self.map_cloud = np.empty((0, 2), dtype=np.float32)

        # plotting
        self._enable_plot = enable_live_plot
        self._fig = self._ax = self._sc = None
        self._plot_xlim = plot_xlim
        self._plot_ylim = plot_ylim

    # -------------------------------------------------------------
    @staticmethod
    def _wrap(a):
        """Normaliza ángulo a [-π, π]."""
        return (a + np.pi) % (2*np.pi) - np.pi

    # -------------------------------------------------------------
    def update(self):
        """Actualiza la pose integrando los encoders."""
        L_now = self.sim.getJointPosition(self.hL)
        R_now = self.sim.getJointPosition(self.hR)

        # Desenvuelve los ángulos para evitar saltos ±2π
        dphi_L = ((L_now - self.prev_L + np.pi) % (2*np.pi)) - np.pi
        raw_R  = (R_now - self.prev_R)
        dphi_R = ((raw_R + np.pi) % (2*np.pi) - np.pi) * self.bias_R
        self.prev_L, self.prev_R = L_now, R_now

        d_sL = self.R * dphi_L
        d_sR = self.R * dphi_R
        d_theta = (d_sR - d_sL) / self.B
        d_s = (d_sR + d_sL) * 0.5

        self.x += d_s * np.cos(self.th + d_theta * 0.5)
        self.y += d_s * np.sin(self.th + d_theta * 0.5)
        self.th = self._wrap(self.th + d_theta)

        return self.x, self.y, self.th

    # -------------------------------------------------------------
    def integrate_points(self, pts_local, *, sensor=None, pause: float = 0.001, ax=None):
        """Transforma puntos locales al mapa y los añade a la nube global.

        Si se pasa `ax`, los puntos se dibujan en ese subplot en vez de usar
        la figura propia de OdometrySensor. Esto permite reutilizar el subplot
        compartido creado por HokuyoSensor.
        """
        import numpy as np
        if pts_local.size == 0:
            return

        # --- 1. si se pasa el sensor Hokuyo, compensar su offset/yaw ---
        if sensor is not None:
            c_off, s_off = np.cos(sensor.off_yaw), np.sin(sensor.off_yaw)
            R_off = np.array([[c_off, -s_off], [s_off, c_off]])
            pts_local = (pts_local @ R_off.T) + np.array([sensor.off_x, sensor.off_y])

        # --- 2. transformar al marco global usando la pose odométrica ---
        c, s = np.cos(self.th), np.sin(self.th)
        R = np.array([[c, -s], [s,  c]])
        pts_map = (pts_local @ R.T) + np.array([self.x, self.y])
        self.map_cloud = np.vstack((self.map_cloud, pts_map))

        # --- Ruta 1: subplot externo compartido -----------------------
        if ax is not None:
            import matplotlib.pyplot as plt
            if not hasattr(self, '_ax_shared') or self._ax_shared is not ax:
                # primera vez con este axes
                self._ax_shared = ax
                self._sc_shared = ax.scatter(self.map_cloud[:, 0], self.map_cloud[:, 1],
                                             s=1, alpha=0.3)
                ax.set_aspect('equal')
            else:
                self._sc_shared.set_offsets(self.map_cloud)

            if ax.figure.canvas is not None:
                ax.figure.canvas.draw_idle()
            plt.pause(pause)
            return

        # --- Ruta 2: figura propia (legacy) ---------------------------
        if not self._enable_plot:
            return

        import matplotlib.pyplot as plt
        if self._fig is None:
            plt.ion()
            self._fig, self._ax = plt.subplots()
            self._sc = self._ax.scatter(self.map_cloud[:, 0], self.map_cloud[:, 1],
                                        s=1, alpha=0.3)
            self._ax.set_aspect('equal')
            self._ax.set_xlim(*self._plot_xlim)
            self._ax.set_ylim(*self._plot_ylim)
            self._ax.set_xlabel('X [m]')
            self._ax.set_ylabel('Y [m]')
            self._ax.set_title('Mapa acumulado (odometría)')
        else:
            self._sc.set_offsets(self.map_cloud)
            if self._fig.canvas is not None:
                self._fig.canvas.draw_idle()

        plt.pause(pause)

    # -------------------------------------------------------------
    def get_pose(self):
        """Devuelve la pose actual estimada (x, y, theta)."""
        return self.x, self.y, self.th
