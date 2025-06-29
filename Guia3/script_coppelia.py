import time
import math
# Si deseas backend headless, descomenta las dos líneas siguientes.  Para ver
# las ventanas en tiempo real y además guardar archivos, mantenlas comentadas.
# import matplotlib
# matplotlib.use('Agg')
from components.ekf_slam import EKFSLAM
from components.sensors  import IMUSensor, GPSSensor, HokuyoSensor, OdometrySensor
from components.simulation import SimulationEnvironment
from components.robot_controller import RobotController
from components.visualization import TrajectoryVisualizer, hokuyo_plot_map
import numpy as np
import threading
from threading import Event
import os

from campos_potenciales import potential_field_planning
import campos_potenciales
from config_campos_potenciales import get_config, print_config

# Bloques de coppelia como landmarks
BLOCK_ALIASES = [
    '/ConcretBlock[0]',
    '/ConcretBlock[1]',
    '/ConcretBlock[2]',
    '/ConcretBlock[3]',
    '/ConcretBlock[4]',
]

def optimize_waypoints(waypoints, min_distance=1.0):
    """
    Optimiza la lista de waypoints eliminando puntos muy cercanos para mejorar el control.
    
    Args:
        waypoints: Lista de waypoints [[x, y, z], ...]
        min_distance: Distancia mínima entre waypoints consecutivos
    
    Returns:
        Lista optimizada de waypoints
    """
    if len(waypoints) <= 2:
        return waypoints
    
    optimized = [waypoints[0]]  # Siempre incluir el primer waypoint
    
    for i in range(1, len(waypoints)):
        # Calcular distancia al último waypoint agregado
        last_wp = optimized[-1]
        current_wp = waypoints[i]
        
        dx = current_wp[0] - last_wp[0]
        dy = current_wp[1] - last_wp[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Solo agregar si está suficientemente lejos o es el último waypoint
        if distance >= min_distance or i == len(waypoints) - 1:
            optimized.append(current_wp)
    
    return optimized

def validate_trajectory_continuity(waypoints, max_jump=5.0):
    """
    Valida que la trayectoria sea continua y no tenga saltos grandes.
    
    Args:
        waypoints: Lista de waypoints
        max_jump: Distancia máxima permitida entre waypoints consecutivos
    
    Returns:
        bool: True si la trayectoria es válida
    """
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i-1][0]
        dy = waypoints[i][1] - waypoints[i-1][1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > max_jump:
            print(f"⚠️  Salto detectado entre waypoints {i} y {i+1}: {distance:.2f}m > {max_jump}m")
            return False
    
    return True


def main():
    """Main function to run the robot simulation with potential field navigation."""
    print('Program started - Potential Field Navigation')
    
    sim_env = SimulationEnvironment()
    # Initialize simulation
    block_handles, block_positions = {}, {}
    for alias in BLOCK_ALIASES:
        try:
            h   = sim_env.sim.getObject(alias)            # path + alias
            pos = sim_env.sim.getObjectPosition(h, -1)[:2]
            block_handles[alias] = h
            block_positions[h]  = pos
            print(f"✓ {alias} registrado → {pos}")
        except Exception as e:
            print(f"⚠️  {alias} no encontrado: {e}")
            
    # 2. asignación dinámica de IDs EKF
    ekf_id_from_handle = {}   # {handle → id_EKF}
    next_lm_id = 0            # contador incremental
    
    def block_landmarks(pose_xy, theta, max_range=4.0):
        nonlocal next_lm_id
        x, y = pose_xy
        lm = []

        for h, (bx, by) in block_positions.items():
            dx, dy = bx - x, by - y
            r = math.hypot(dx, dy)
            if r < max_range:
                beta = math.atan2(dy, dx) - theta
                beta = (beta + math.pi) % (2*math.pi) - math.pi  # wrap [-π,π]

                # asignar id estable la primera vez
                if h not in ekf_id_from_handle:
                    ekf_id_from_handle[h] = next_lm_id
                    print(f"➕ Nuevo landmark id={next_lm_id} ← bloque handle={h}")
                    next_lm_id += 1

                lm_id = ekf_id_from_handle[h]
                lm.append((r, beta, lm_id))
        return lm

    # Get handles for all required objects
    handles = sim_env.get_handles({
        'left_motor': '/PioneerP3DX/leftMotor',
        'right_motor': '/PioneerP3DX/rightMotor',
        'robot': '/PioneerP3DX',
        'gps': '/PioneerP3DX/GPS',
        'gps2': '/PioneerP3DX2/GPS2', 
        'fastHokuyo': '/PioneerP3DX/fastHokuyo',
        # 'ranges_sig': "fastHokuyo_scanRangesB64",
        # 'gyro_sig': "gyroDataB64",
        # 'accel_sig': "accelDataB64",
        # 'gps_sig': "gpsDataB64"

    })
    
    # Print handles for debugging
    for name, handle in handles.items():
        print(f"Handle {name}: {handle}")
    


    imu_sensor = IMUSensor(sim_env.sim,
                       gyro_handle='gyroZ',
                       acc_handle='accX',
                       gyro_noise_deg=0.15,    # ruido razonable
                       acc_noise=0.3)

    gps_sensor = GPSSensor(
        sim_env.sim,
        handles['gps'],
        sigma_xy=0.05,
        sigma_yaw=0.01,  # ruido de orientación
        freq_hz=5,
        robot_handle=handles['robot'],
        trailer_handle=handles['gps2']     # si quieres la pose del trailer
    )

    hokuyo = HokuyoSensor(
        sim_env.sim,
        handles['fastHokuyo'],
        ranges_sig='fastHokuyo_scanRangesB64',
        angles_sig='fastHokuyo_scanAnglesB64',
        sensor_offset_xy=(0, 0.00),       # ↙ calibra con una pared recta
        sensor_yaw_offset_deg=0,          # láser mirando +Y del robot
    )

    # ------------------------------ Odometry sensor --------------------
    # Pose inicial verdadera para alinear marco odométrico
    init_pose_robot = sim_env.sim.getObjectPose(handles['robot'], -1)
    init_yaw_robot = math.atan2(
        2*(init_pose_robot[6]*init_pose_robot[5] + init_pose_robot[3]*init_pose_robot[4]),
        1 - 2*(init_pose_robot[4]**2 + init_pose_robot[5]**2)
    )

    odom_sensor = OdometrySensor(
        sim_env.sim,
        handles['left_motor'],
        handles['right_motor'],
        init_x=init_pose_robot[0],
        init_y=init_pose_robot[1],
        init_yaw=init_yaw_robot,
        bias_right=1.001,
        enable_live_plot=False,
    )

    first_fix = None
    while first_fix is None:                # loop hasta que el GPS dispare
        first_fix = gps_sensor.read_if_ready()

    init_x, init_y = first_fix              # ← solo 2 valores (x, y)
    init_th       = 0.0                     # heading inicial (puedes dejar 0 rad)
    # ------------------------------------------------------------------

    ekf = EKFSLAM(
    init_pose=[init_x, init_y, init_th],
    Q_base=np.diag([0.04, 0.04, (2*np.pi/180)**2]),
    R_gps=np.diag([0.001, 0.001]),
    R_gyro=np.array([[ (0.001*np.pi/180)**2 ]]),
    R_lidar=np.diag([0.15**2, (5*np.pi/180)**2]),
    dt=0.1)

    # Initialize robot controller
    robot = RobotController(sim_env, gps_sensor)
    
    # Initialize visualizer
    visualizer = TrajectoryVisualizer()
    config = get_config()
    
    # Parámetros para la planificación de campos potenciales
    start_x = config['start_x']
    start_y = config['start_y']
    goal_x = config['goal_x']
    goal_y = config['goal_y']
    grid_size = config['grid_size']
    robot_width = config['robot_width']      
    robot_length = config['robot_length']    
    altura = config['waypoint_height']
    
    # Obstáculos del entorno
    obstacle_x = config['obstacle_x']
    obstacle_y = config['obstacle_y']
    
    campos_potenciales.show_animation = config['show_animation']
    
    sim_env.start()

    initial_position = gps_sensor.get_position()
    print(f"\nPosición inicial del robot: [{initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f}]")

    # Ajustar origen dinámico si se solicita
    if config['use_dynamic_start']:
        start_x, start_y = initial_position[0], initial_position[1]
        print(f"Usando posición inicial ajustada: ({start_x:.2f}, {start_y:.2f})")

    path_x, path_y = potential_field_planning(
        start_x, start_y, goal_x, goal_y,
        obstacle_x, obstacle_y, grid_size, robot_width, robot_length, show_animation=False
    )

    waypoints = [[x, y, altura] for x, y in zip(path_x, path_y)]
    waypoints = optimize_waypoints(waypoints, min_distance=0.8)

    if config['max_waypoints'] and len(waypoints) > config['max_waypoints']:
        step = max(1, len(waypoints) // config['max_waypoints'])
        waypoints = waypoints[::step]
        if waypoints[-1] != [path_x[-1], path_y[-1], altura]:
            waypoints.append([path_x[-1], path_y[-1], altura])

        if not validate_trajectory_continuity(waypoints, max_jump=3.0):
            print("⚠️  Trayectoria discontinua - podrías interpolar si lo deseas")
        print(f"Trayectoria generada: {len(waypoints)} waypoints")


    nav_done = Event()  # para coordinar fin del worker

    def navigation_worker():
        try:
            start_time = sim_env.get_simulation_time()
            robot.set_start_time(start_time)

            # Guardar última medición GPS válida (para visualización)
            last_gps_position = first_fix  # primera fijación

            # Tolerancia para waypoints
            original_max_error = robot.max_error
            robot.max_error = 1.5

            successful_waypoints = 0
            for waypoint_idx, target_position in enumerate(waypoints):
                print(f"\nNavigando hacia waypoint {waypoint_idx+1}/{len(waypoints)}: "
                      f"[{target_position[0]:.2f}, {target_position[1]:.2f}]")

                if waypoint_idx == len(waypoints) - 1:
                    robot.max_error = 0.5
                    print("🎯 Waypoint final - usando tolerancia estricta")

                max_attempts = 200
                attempts = 0

                while attempts < max_attempts:

                    # Leer sensores
                    imu_data = imu_sensor.read()

                    # Fusión de medidas (IMU + GPS + Landmarks)
                    pose_hat, pose_gt, gps_new = ekf.fuse_measurements(
                        gps_sensor,
                        imu_data,
                        block_landmarks,
                    )

                    # -------------- Backup GPS para visualización -------------
                    if gps_new is not None:
                        last_gps_position = gps_new
                    gps_position = last_gps_position

                    # Datos de posición para métricas
                    real_position = gps_sensor.get_real_position()
                    trailer_position = gps_sensor.get_trailer_position()

                    # ----- Point-cloud almacenada -----
                    cloud = hokuyo.get_xy_world()
                    visualizer.record_pointcloud(cloud)

                    # ------------------- ODOMETRÍA + MAPA -------------------
                    robot_pose = sim_env.sim.getObjectPose(handles['robot'], -1)
                    pts_local = hokuyo.world_to_vehicle(cloud, robot_pose)
                    odom_pose = odom_sensor.update()  # integra encoders
                    odom_sensor.integrate_points(pts_local, sensor=hokuyo, ax=hokuyo._ax_map_ax)

                    # --- Visualización en vivo de la nube Hokuyo ---
                    try:
                        # Pose 2-D + yaw usando GPS+IMU/gyro (simplificado)
                        pos = gps_sensor.get_position()
                        yaw = gps_sensor.get_orientation()[2]
                        veh_pose = (pos[0], pos[1], yaw)
                        hokuyo.plot_local(veh_pose)
                        hokuyo.plot_map(subsample=3)
                    except Exception as e:
                        # En caso de error (p.ej. el simulador se cerró) sigue sin interrumpir navegación
                        print(f"⚠️  Hokuyo plot error: {e}")

                    waypoint_reached = robot.navigate_to_waypoint(
                        target_position, pose_override=pose_hat
                    )

                    visualizer.record_data(
                        elapsed_time=sim_env.get_simulation_time() - start_time,
                        real_position=real_position,
                        gps_position=gps_position,
                        trailer_position=trailer_position,
                        ekf_position=pose_hat,
                        odom_position=odom_pose,
                        real_yaw=yaw,
                    )

                    if waypoint_reached:
                        visualizer.record_waypoint_reached(sim_env.get_simulation_time() - start_time)
                        successful_waypoints += 1
                        print(f"✓ Waypoint {waypoint_idx+1} alcanzado en {attempts} intentos!")
                        break
                    attempts += 1

            success_rate = (successful_waypoints / len(waypoints)) * 100
            print(f"\n📊 Estadísticas de navegación:\n   Waypoints alcanzados: {successful_waypoints}/{len(waypoints)} ({success_rate:.1f}%)")

        except KeyboardInterrupt:
            print("Navigation interrupted")
        finally:
            robot.max_error = original_max_error
            # Detener motores
            if 'left_motor' in handles and 'right_motor' in handles:
                sim_env.set_joint_velocity(handles['left_motor'], 0)
                sim_env.set_joint_velocity(handles['right_motor'], 0)

            nav_done.set()

    # -----------------------
    # Lanzar hilo
    nav_thread = threading.Thread(target=navigation_worker, daemon=True)
    nav_thread.start()
    nav_thread.join()

    # Generar y guardar gráficos de trayectoria, errores, etc.
    visualizer.plot_results(waypoints, show=False)

    # Guardar el mapa odométrico en PDF vectorizado
    outputs_img_dir = os.path.join('outputs', 'images')
    os.makedirs(outputs_img_dir, exist_ok=True)
    # Crear la figura de mapa en el hilo principal y luego guardarla
    hokuyo_plot_map(hokuyo, pause=0)
    from components.visualization import hokuyo_save_map_pdf
    hokuyo_save_map_pdf(hokuyo, os.path.join(outputs_img_dir, 'map_odom.pdf'), odom_cloud=odom_sensor.map_cloud)

    # Cerrar todas las figuras abiertas (gestión desde el hilo principal)
    import matplotlib.pyplot as plt
    plt.close('all')

    sim_env.stop()
    print('Program finished')


if __name__ == "__main__":
    main() 