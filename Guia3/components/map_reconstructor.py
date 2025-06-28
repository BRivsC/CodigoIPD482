"""components/map_reconstructor.py

Módulo para la reconstrucción (online) de mapas 2D a partir de los datos
LIDAR Hokuyo, IMU y GPS del PioneerP3DX en CoppeliaSim.

La lógica está encapsulada en una clase `MapReconstructor` para que pueda
ser reutilizada desde otros scripts (p.ej. *script_coppelia.py*) o bien
invocarse directamente mediante `python -m components.map_reconstructor`.

El código es una refactorización del script monolítico proporcionado por
el usuario, preservando su funcionalidad original pero adaptándolo a una
arquitectura orientada a objetos y a los lineamientos de este proyecto
(SRP, alta cohesión, documentación, etc.).
"""

from __future__ import annotations

import base64
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

__all__ = ["MapReconstructor", "Pose"]


@dataclass(slots=True)
class Pose:
    """Representa la pose 2D (x, y, theta) del robot en el marco global."""

    x: float
    y: float
    theta: float  # yaw [rad]


class MapReconstructor:
    """Reconstrucción de mapa en línea usando sensores de CoppeliaSim.

    Parámetros
    ----------
    robot_path : str, default '/PioneerP3DX'
        Ruta (scene hierarchy) del objeto base del robot.
    lidar_path : str, default '/PioneerP3DX/fastHokuyo'
        Ruta del sensor Hokuyo.
    gps_sig : str, default 'gpsDataB64'
        Nombre de la *string signal* publicada por el script Lua del GPS.
    gyro_sig : str, default 'gyroDataB64'
        Señal con los datos de giroscopio (float32[3]). Se usa el componente Z.
    accel_sig : str, default 'accelDataB64'
        Señal con la aceleración (float32[3]). Se usa el plano xy.
    max_scan_distance : float, default 5.0
        Distancia máxima (m) que reporta el LIDAR.
    enable_live_plot : bool, default True
        Indica si se debe mostrar la gráfica interactiva en tiempo real.
    sensor_offset_xy : tuple[float, float], default (0.0, 0.0)
        Offset de posición (x, y) del sensor en el marco del robot.
    sensor_yaw_offset_deg : float, default 0.0
        Offset de orientación (yaw) del sensor en grados.
    min_contact_dist : float | None, default None
        Distancia mínima para considerar un punto válido (m). Si None, se usa el criterio estándar de `max_scan_distance`.
    subsample : int, default 1
        Submuestreo: almacenar solo 1 de cada `subsample` puntos válidos.
    """

    def __init__(
        self,
        sim,
        imu_sensor: IMUSensor,
        gps_sensor: GPSSensor,
        hokuyo_sensor: HokuyoSensor,
        max_scan_distance: float = 5.0,
        *,
        enable_live_plot: bool = True,
        sensor_offset_xy: tuple[float, float] = (0.0, 0.0),
        sensor_yaw_offset_deg: float = 0.0,
        min_contact_dist: float | None = None,
        subsample: int = 1,
    ) -> None:
        # --- Conexión a CoppeliaSim -------------------------------------

        # Distancia máxima para filtrar puntos del LIDAR
        self._max_scan_dist = max_scan_distance

        # Offset geométrico del sensor (en el marco del robot)
        self._off_x, self._off_y = sensor_offset_xy
        self._off_yaw = math.radians(sensor_yaw_offset_deg)

        # --- Estructuras de estado --------------------------------------
        self.running = False  # bandera de loop principal
        self._set_sigint_handler()

        self._point_cloud: List[Tuple[float, float]] = []
        self._odom_trace: List[Tuple[float, float]] = []
        self._inertial_trace: List[Tuple[float, float]] = []
        self._gps_trace: List[Tuple[float, float]] = []

        # Pose estimada por integración inercial
        #imu_data = [x, y, theta]
        p0 = imu_data[0:1]
        init_theta = imu_data[2]

        self._est_pose = Pose(p0[0], p0[1], init_theta)
        self._est_vel = np.zeros(2, dtype=np.float32)  # vx, vy
        self._last_t = 0.0

        # Leer ángulos del LIDAR una sola vez
        packed_ang_b64 = self._sim.getStringSignal(self.ANGLES_SIG)
        if packed_ang_b64 is None:
            sys.stderr.write(
                "⚠️  No llegó la señal de ángulos del Hokuyo. Se usará linspace.\n"
            )
            self._angles = np.linspace(-np.pi, np.pi, 1081, dtype=np.float32)
        else:
            self._angles = np.frombuffer(
                base64.b64decode(packed_ang_b64), dtype=np.float32
            )
        self._num_samples = self._angles.size

        # Guardo banderas de control
        self._enable_plot = enable_live_plot

        # Contador de frames para refresco de la GUI
        self._refresh_counter = 0
        self._min_contact_dist = min_contact_dist
        self._subsample = max(1, int(subsample))

        if self._enable_plot:
            self._init_realtime_plot()

    # ---------------------------------------------------------------------
    # Propiedades públicas
    # ---------------------------------------------------------------------
    @property
    def point_cloud(self) -> np.ndarray:
        """Devuelve la nube de puntos acumulada (N,2) como ndarray."""
        if self._point_cloud:
            return np.asarray(self._point_cloud, dtype=np.float32)
        return np.empty((0, 2), dtype=np.float32)

    @property
    def odom_trace(self) -> np.ndarray:
        if self._odom_trace:
            return np.asarray(self._odom_trace, dtype=np.float32)
        return np.empty((0, 2), dtype=np.float32)

   


    # .................................................................
    # Gráfica interactiva
    def _init_realtime_plot(self) -> None:
        plt.ion()
        self._fig, self._ax = plt.subplots()
        self._scat = self._ax.scatter([], [], s=1, alpha=0.25, label="mapa estimado")
        (self._odom_line,) = self._ax.plot([], [], "r-", label="verdadero (odom)")
        (self._imu_line,) = self._ax.plot([], [], "b--", label="imu dead-reckoning")
        self._gps_scat = self._ax.scatter([], [], c="g", s=6, marker="x", label="GPS")
        # Límites fijos solicitados
        self._ax.set_xlim(-12, 10)
        self._ax.set_ylim(-12, 10)

        self._ax.set_xlabel("x [m]")
        self._ax.set_ylabel("y [m]")
        self._ax.set_title("Reconstrucción en tiempo real")
        self._ax.set_aspect("equal")
        self._ax.legend()
        plt.show(block=False)

    # .................................................................
    # Loop principal
    def _main_loop(self) -> None:
        while self.running:
            self._capture_step()
            time.sleep(0.02)  # ~50 Hz
            if self._enable_plot:
                if self._refresh_counter % 10 == 0:
                    self._update_plot()
                self._refresh_counter += 1

        if self._enable_plot:
            self._show_final_map()

    # .................................................................
    def _capture_step(self) -> None:
        # --- Pose ground-truth (odom) ---------------------------------
        x, y = self._est_pose.x, self._est_pose.y
        yaw = self._est_pose.theta
        pose_gt = Pose(x, y, yaw)
        self._odom_trace.append((x, y))

        # --- Integración inercial (dead-reckoning) --------------------
        t_now = self.sim_time()
        dt = max(t_now - self._last_t, 1e-3)

        gyro_b64 = self._sim.getStringSignal(self.GYRO_SIG)
        accel_b64 = self._sim.getStringSignal(self.ACCEL_SIG)
        if gyro_b64 and accel_b64:
            wz = np.frombuffer(base64.b64decode(gyro_b64), dtype=np.float32)[2]
            ax, ay, _ = np.frombuffer(base64.b64decode(accel_b64), dtype=np.float32)

            # Integrar orientación
            self._est_pose.theta += float(wz) * dt

            # Rotar aceleración al marco mundo
            c, s = math.cos(self._est_pose.theta), math.sin(self._est_pose.theta)
            ax_w, ay_w = c * ax - s * ay, s * ax + c * ay

            # Integrar velocidad y posición
            self._est_vel[0] += ax_w * dt
            self._est_vel[1] += ay_w * dt
            self._est_pose.x += self._est_vel[0] * dt
            self._est_pose.y += self._est_vel[1] * dt

        self._inertial_trace.append((self._est_pose.x, self._est_pose.y))

        # --- LIDAR -----------------------------------------------------
        packed_ranges_b64 = self._sim.getStringSignal(self.RANGES_SIG)
        if packed_ranges_b64 is not None:
            ranges = np.frombuffer(base64.b64decode(packed_ranges_b64), dtype=np.float32)
            if ranges.size == self._num_samples:
                mask = ranges < (self._max_scan_dist * 0.9999)
                if mask.any():
                    valid_ranges = ranges[mask]
                    valid_angles = self._angles[mask]
                    # ------------------ FILTRADO OPCIONAL --------------------
                    if self._min_contact_dist is not None:
                        contact_mask = valid_ranges <= self._min_contact_dist
                        if not contact_mask.any():
                            return  # no se encontraron puntos cercanos
                        valid_ranges = valid_ranges[contact_mask]
                        valid_angles = valid_angles[contact_mask]
                    # Submuestreo simple para no abarrotar el mapa
                    if self._subsample > 1 and valid_ranges.size > 0:
                        step = self._subsample
                        valid_ranges = valid_ranges[::step]
                        valid_angles = valid_angles[::step]
                    # ----------------------------------------------------------
                    # Posición y orientación del LIDAR en mundo
                    c_r, s_r = math.cos(pose_gt.theta), math.sin(pose_gt.theta)
                    lidar_x = pose_gt.x + c_r * self._off_x - s_r * self._off_y
                    lidar_y = pose_gt.y + s_r * self._off_x + c_r * self._off_y

                    # Ángulo global de cada rayo (robot θ + offset + ángulo relativo)
                    global_th = pose_gt.theta + self._off_yaw + valid_angles

                    xs = lidar_x + valid_ranges * np.cos(global_th)
                    ys = lidar_y + valid_ranges * np.sin(global_th)
                    self._point_cloud.extend(zip(xs, ys))

        # --- GPS opcional --------------------------------------------
        gps_b64 = self._sim.getStringSignal(self.GPS_SIG)
        if gps_b64:
            gps = np.frombuffer(base64.b64decode(gps_b64), dtype=np.float32)
            self._gps_trace.append((gps[0], gps[1]))

        self._last_t = t_now

    # .................................................................
    # Actualización de la gráfica en tiempo real
    def _update_plot(self) -> None:
        if self._point_cloud:
            self._scat.set_offsets(np.asarray(self._point_cloud, dtype=np.float32))
        if self._odom_trace:
            od = np.asarray(self._odom_trace, dtype=np.float32)
            self._odom_line.set_data(od[:, 0], od[:, 1])
        if self._inertial_trace:
            it = np.asarray(self._inertial_trace, dtype=np.float32)
            self._imu_line.set_data(it[:, 0], it[:, 1])
        if self._gps_trace:
            gp = np.asarray(self._gps_trace, dtype=np.float32)
            self._gps_scat.set_offsets(gp)
        # No cambiar límites: se mantienen fijos (-12,10)
        plt.pause(0.001)

    def _show_final_map(self) -> None:
        if not self._enable_plot or not self._point_cloud:
            print("⚠️  No se acumularon puntos - ¿está publicando el Hokuyo?")
            return

        pc = self.point_cloud
        od = self.odom_trace

        plt.ioff()
        plt.figure()
        plt.scatter(pc[:, 0], pc[:, 1], s=1, alpha=0.25, label="mapa estimado")
        plt.plot(od[:, 0], od[:, 1], "r-", label="verdadero (odom)")
        if self._inertial_trace:
            it = np.asarray(self._inertial_trace, dtype=np.float32)
            plt.plot(it[:, 0], it[:, 1], "b--", label="imu dead-reckoning")
        if self._gps_trace:
            gp = np.asarray(self._gps_trace, dtype=np.float32)
            plt.scatter(gp[:, 0], gp[:, 1], c="g", s=6, marker="x", label="GPS")
        plt.gca().set_aspect("equal")
        plt.title("Reconstrucción con IMU vs ground-truth")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.tight_layout()
        plt.show()

