# ekf_slam.py  – EKF-SLAM con IMU (gyro+acc), GPS y Hokuyo
import numpy as np

DEG2RAD = np.pi / 180.0

class EKFSLAM:
    """
    Estado   x = [x, y, theta, m1x, m1y, m2x, m2y, …]ᵀ
    Predicción  :se estima velocidad lineal como v = v_prev + a_x*dt (ruido alto), usando el acelerómetro, se actualiza la posición y orientación del robot.    
    Predicción  : se usa el modelo unicycle (x,y,theta) con velocidad lineal estimada.
    Corrección  : Se utiliza datos de gps GPS   (x,y)
                    : Gyro (yaw) 
                    : Hokuyo landmarks   (r, β)
    """
    def __init__(self, init_pose, Q_base, R_gps, R_gyro, R_lidar, dt=0.1):
        self.dt  = dt
        self.x   = np.asarray(init_pose, dtype=float)           # len=3
        self.P   = np.diag([1.0, 1.0, (10*DEG2RAD)**2])         # incertidumbre alta
        self.Q_b = Q_base                                       # 3×3 base
        self.Rg  = R_gps                                        # 2×2
        self.Ryaw= R_gyro                                       # 1×1
        self.Rl  = R_lidar                                      # 2×2
        self.v_est = 0.0                                        # velocidad lineal filtrada

    @staticmethod
    def _wrap(a):                    # wrap angle to [-π,π]
        return (a + np.pi) % (2*np.pi) - np.pi

    def _F_jacobian(self):
        _, _, th = self.x[:3]
        c, s = np.cos(th), np.sin(th)
        return np.array([[1, 0, -self.v_est*self.dt*s],
                         [0, 1,  self.v_est*self.dt*c],
                         [0, 0,  1]])

    def predict(self, gyro_z, acc_x):
        # estimar velocidad lineal con acelerómetro
        self.v_est += acc_x * self.dt
        # actualizar estado con las mediciones del acelerometro (unicycle)
        x, y, th = self.x[:3]
        self.x[0] = x + self.v_est * self.dt * np.cos(th)
        self.x[1] = y + self.v_est * self.dt * np.sin(th)
        self.x[2] = self._wrap(th + gyro_z * self.dt)

        # Jacobiano F y covarianza proceso
        F = self._F_jacobian()
        v_scale = max(abs(self.v_est), 0.01)
        Q = self.Q_b * v_scale**2            
        dim = self.P.shape[0]

        F_big = np.eye(dim)
        F_big[:3, :3] = F
        Q_big = np.zeros((dim, dim))
        Q_big[:3, :3] = Q

        self.P = F_big @ self.P @ F_big.T + Q_big  # covarianza del proceso 

    def update_gps(self, z_xy):
        H = np.zeros((2, self.x.size))
        H[:,:2] = np.eye(2)
        self._ekf_update(z_xy, H, self.Rg)

    def update_yaw(self, z_theta):
        H = np.zeros((1, self.x.size))
        H[0,2] = 1
        self._ekf_update(np.asarray([z_theta]), H, self.Ryaw)

    def update_landmarks(self, z_list):
        """
        z_list = [(r, beta, id), ...]  – id es índice int del landmark.
        Si id == -1   => landmark nuevo, se añade al estado.
        """
        for r, beta, lm_id in z_list:
            needed = 3 + 2*lm_id + 2     # índice final que necesitamos
            if lm_id == -1 or needed > self.x.size:
                # id = -1   O   id todavía no existe → créalo
                lm_x = self.x[0] + r*np.cos(beta + self.x[2])
                lm_y = self.x[1] + r*np.sin(beta + self.x[2])
                self.x = np.hstack([self.x, [lm_x, lm_y]])
                dim   = self.P.shape[0]
                P_new = np.zeros((dim+2, dim+2))
                P_new[:dim,:dim] = self.P
                P_new[dim:,dim:] = np.diag([10.0, 10.0])
                self.P = P_new
                lm_id = (dim-3)//2        # índice recién asignado

            # (aquí ya existe con seguridad) -----------------------------
            lx, ly = self.x[3+2*lm_id : 3+2*lm_id+2]
            dx, dy = lx - self.x[0], ly - self.x[1]
            q  = dx*dx + dy*dy
            sqrt_q = np.sqrt(q)
            z_hat  = np.array([sqrt_q,
                              self._wrap(np.arctan2(dy, dx) - self.x[2])])
            # Jacobiano H_j
            H = np.zeros((2, self.x.size))
            H[0,0] = -dx / sqrt_q;   H[0,1] = -dy / sqrt_q
            H[1,0] =  dy / q;        H[1,1] = -dx / q
            H[1,2] = -1.0
            H[0,3+2*lm_id]     =  dx / sqrt_q
            H[0,3+2*lm_id + 1] =  dy / sqrt_q
            H[1,3+2*lm_id]     = -dy / q
            H[1,3+2*lm_id + 1] =  dx / q
            self._ekf_update(np.array([r, beta]), H, self.Rl)

    def _ekf_update(self, z, H, R):
        y  = z - H @ self.x                     # innovación
        if y.size == 1: y[0] = self._wrap(y[0]) # para el ángulo
        S  = H @ self.P @ H.T + R
        K  = self.P @ H.T @ np.linalg.inv(S)    # ganancia de Kalman
        self.x = self.x + K @ y                 # actualización del estado (correccion xk|k)
        self.x[2] = self._wrap(self.x[2])  
        I = np.eye(self.P.shape[0])             
        self.P = (I - K @ H) @ self.P           # actualización de la covarianza (corrección Pk|k)

    def pose(self):
        return self.x[:3]

    def fuse_measurements(self, gps_sensor, imu_data, block_landmarks):
        """Realiza un ciclo completo de fusión de sensores.

        Parámetros
        ----------
        gps_sensor : GPSSensor
            Instancia del sensor GPS (debe exponer los métodos read_if_ready(),
            get_orientation() y get_position()).
        imu_data : tuple
            Tupla (gyro_z, acc_x) correspondiente a la lectura del IMU.
        block_landmarks : callable
            Función callback que devuelve una lista de medidas de landmarks
            [(r, beta, id), ...] a partir de la pose actual (en el frame del
            robot).

        Retorna
        -------
        pose_hat : ndarray shape (3,)
            Estimación del estado [x, y, theta] después de la fusión.
        pose_gt : list[float]
            Posición real del robot obtenida desde el GPS (sin ruido).
        gps_z   : ndarray or None
            Última medición GPS utilizada. None si no hubo nueva lectura.
        """
        # --- Predicción con la IMU -------------------------------------
        gyro_z, acc_x = imu_data
        self.predict(gyro_z, acc_x)

        # --- Corrección con el giro (yaw) -----------------------------
        yaw = gps_sensor.get_orientation()[2]
        self.update_yaw(yaw)

        # --- Corrección con posición GPS ------------------------------
        gps_z = gps_sensor.read_if_ready()
        if gps_z is not None:
            self.update_gps(gps_z)

        # --- Corrección con landmarks (láser) -------------------------
        pose_hat = self.pose()
        lm_z = block_landmarks(pose_hat[:2], pose_hat[2])
        if lm_z:
            self.update_landmarks(lm_z)

        # Pose real (ground-truth) para diagnóstico / visualización
        pose_gt = gps_sensor.get_position()
        return pose_hat, pose_gt, gps_z
