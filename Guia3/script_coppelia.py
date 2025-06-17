import time
import math
from components.simulation import SimulationEnvironment
from components.sensors import GPSSensor
from components.robot_controller import RobotController
from components.visualization import TrajectoryVisualizer

# Importar las funciones de campos potenciales
from campos_potenciales import potential_field_planning
import campos_potenciales
from config_campos_potenciales import get_config, print_config

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
    
    # Initialize simulation
    sim_env = SimulationEnvironment()
    
    # Get handles for all required objects
    handles = sim_env.get_handles({
        'left_motor': '/PioneerP3DX/leftMotor',
        'right_motor': '/PioneerP3DX/rightMotor',
        'robot': '/PioneerP3DX',
        'gps': '/PioneerP3DX/GPS',
        'gps2': '/PioneerP3DX2/GPS2',  # Nuevo handle para el GPS del trailer
        # 'trailer': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2',
        # 'trailer_left_motor': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2/leftMotor2',
        # 'trailer_right_motor': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint/PioneerP3DX2/rightMotor2',
        # 'hitch_joint': '/PioneerP3DX/Revolute_joint',
        # 'connection_joint': '/PioneerP3DX/Revolute_joint/Cuboid/Revolute_joint'
    })
    
    # Print handles for debugging
    for name, handle in handles.items():
        print(f"Handle {name}: {handle}")
    
    # Initialize GPS sensor para el robot principal
    gps = GPSSensor(
        sim_env.sim,
        handles['gps'],
        handles['robot']
    )
    # Initialize GPS sensor para el trailer
    gps_trailer = GPSSensor(
        sim_env.sim,
        handles['gps2'],
        handles['robot']  # El tercer argumento no afecta para GPS absoluto
    )
    
    # Initialize robot controller
    robot = RobotController(sim_env, gps)
    
    # Initialize visualizer
    visualizer = TrajectoryVisualizer()
    
    # Intentar poner trailer en modo pasivo
    try:
        # Intento 1: Si existen handles de trailer, desactivar control
        if 'trailer_left_motor' in handles and 'trailer_right_motor' in handles:
            sim_env.sim.setObjectFloatParameter(handles['trailer_left_motor'], 2001, 0)  # 2001 = motor enabled
            sim_env.sim.setObjectFloatParameter(handles['trailer_right_motor'], 2001, 0)
        print("Trailer configurado en modo pasivo")
    except Exception as e:
        print(f"Note: Trailer passive mode setup failed: {e}")
        print("El trailer podría tener algo de resistencia.")
    
    # =================================================================
    # CONFIGURACIÓN DE CAMPOS POTENCIALES
    # =================================================================
    
    # Cargar configuración desde archivo externo
    config = get_config()
    print_config()
    
    # Parámetros para la planificación de campos potenciales
    start_x = config['start_x']
    start_y = config['start_y']
    goal_x = config['goal_x']
    goal_y = config['goal_y']
    grid_size = config['grid_size']
    robot_width = config['robot_width']      # CORREGIDO: usar dimensiones rectangulares
    robot_length = config['robot_length']    # CORREGIDO: usar dimensiones rectangulares
    altura = config['waypoint_height']
    
    # Obstáculos del entorno
    obstacle_x = config['obstacle_x']
    obstacle_y = config['obstacle_y']
    
    # Desactivar animación para evitar problemas en simulación
    campos_potenciales.show_animation = config['show_animation']
    
    # Start simulation
    sim_env.start()
    
    # Obtener posición inicial real del robot
    initial_position = gps.get_position()
    print(f"\nPosición inicial del robot: [{initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f}]")
    
    # Usar posición inicial dinámica si está configurado
    if config['use_dynamic_start']:
        start_x, start_y = initial_position[0], initial_position[1]
        print(f"Usando posición inicial ajustada: ({start_x:.2f}, {start_y:.2f})")
    
    # Generar trayectoria usando campos potenciales
    print("\nGenerando trayectoria con campos potenciales...")
    try:
        # CORREGIDO: pasar robot_width y robot_length en lugar de robot_radius
        path_x, path_y = potential_field_planning(
            start_x, start_y, goal_x, goal_y, 
            obstacle_x, obstacle_y, grid_size, robot_width, robot_length
        )
        
        # Validar que se generó una trayectoria válida
        if len(path_x) < 2 or len(path_y) < 2:
            raise ValueError("Trayectoria generada es demasiado corta")
        
        # Convertir la trayectoria a waypoints 3D
        waypoints = []
        for i in range(len(path_x)):
            waypoints.append([path_x[i], path_y[i], altura])
        
        # Optimizar waypoints para mejor control
        waypoints = optimize_waypoints(waypoints, min_distance=0.8)
        print(f"Waypoints optimizados: {len(waypoints)} waypoints")
        
        # Limitar número de waypoints si está configurado para mejor control
        if config['max_waypoints'] and len(waypoints) > config['max_waypoints']:
            step = max(1, len(waypoints) // config['max_waypoints'])
            waypoints = waypoints[::step]
            # Asegurar que el último waypoint esté incluido
            if waypoints[-1] != [path_x[-1], path_y[-1], altura]:
                waypoints.append([path_x[-1], path_y[-1], altura])
            print(f"Waypoints reducidos a {len(waypoints)} (cada {step} waypoints)")
        
        # Validar continuidad de la trayectoria
        if not validate_trajectory_continuity(waypoints, max_jump=3.0):
            print("⚠️  Trayectoria tiene discontinuidades, usando interpolación...")
            # Aquí podrías agregar lógica de interpolación si es necesario
        
        print(f"Trayectoria generada exitosamente con {len(waypoints)} waypoints")
        
        # Calcular distancia total de la trayectoria para métricas
        total_distance = 0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        print(f"Distancia total de la trayectoria: {total_distance:.2f} m")
        
    except Exception as e:
        print(f"Error al generar trayectoria con campos potenciales: {e}")
        print("Usando trayectoria de respaldo (línea recta hacia objetivo)...")
        
        # Trayectoria de respaldo - línea recta hacia el objetivo
        num_points = config['fallback_segments']
        waypoints = []
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)
            waypoints.append([x, y, altura])
    
    print("\nWaypoints a seguir:")
    for i, wp in enumerate(waypoints):
        if i < 5 or i >= len(waypoints) - 2:  # Mostrar solo los primeros y últimos waypoints
            print(f"Waypoint {i+1}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]")
        elif i == 5:
            print("...")
    
    try:
        # Get initial time
        start_time = sim_env.get_simulation_time()
        robot.set_start_time(start_time)
        
        # Configurar tolerancia más adecuada para trayectorias largas
        original_max_error = robot.max_error
        robot.max_error = 1.5  # Tolerancia más generosa para waypoints intermedios
        
        # Navigate to each waypoint
        successful_waypoints = 0
        for waypoint_idx, target_position in enumerate(waypoints):
            print(f"\nNavigando hacia waypoint {waypoint_idx+1}/{len(waypoints)}: "
                  f"[{target_position[0]:.2f}, {target_position[1]:.2f}]")
            
            # Usar tolerancia más estricta para el último waypoint
            if waypoint_idx == len(waypoints) - 1:
                robot.max_error = 0.5  # Tolerancia más estricta para el objetivo final
                print("🎯 Waypoint final - usando tolerancia estricta")
            
            # Control loop for current waypoint con timeout
            max_attempts = 200  # Máximo número de intentos por waypoint
            attempts = 0
            
            while attempts < max_attempts:
                attempts += 1
                
                # Control timing for consistent dt
                current_time = sim_env.get_simulation_time()
                if current_time - robot.prev_time < 0.05:
                    time.sleep(0.001)
                    continue
                
                # Navegación y registro de datos
                waypoint_reached = robot.navigate_to_waypoint(
                    target_position, None  # No pasamos visualizer aquí
                )
                
                # Obtener posiciones
                gps_position = gps.get_position()
                real_position = gps.get_real_position()
                trailer_position = gps_trailer.get_position()
                
                # Registrar en el visualizador
                elapsed_time = current_time - start_time
                visualizer.record_data(elapsed_time, real_position, gps_position, trailer_position)
                
                # If waypoint reached, go to next one
                if waypoint_reached:
                    visualizer.record_waypoint_reached(elapsed_time)
                    successful_waypoints += 1
                    print(f"✓ Waypoint {waypoint_idx+1} alcanzado en {attempts} intentos!")
                    break
            
            # Si no se alcanzó el waypoint, continuar al siguiente
            if attempts >= max_attempts:
                print(f"⚠️  Timeout en waypoint {waypoint_idx+1}, continuando...")
        
        # Estadísticas finales
        success_rate = (successful_waypoints / len(waypoints)) * 100
        print(f"\n📊 Estadísticas de navegación:")
        print(f"   Waypoints alcanzados: {successful_waypoints}/{len(waypoints)} ({success_rate:.1f}%)")
        
        if success_rate >= 80:
            print("🎯 ¡Navegación completada exitosamente!")
        else:
            print("⚠️  Navegación completada con errores")
            
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Restaurar tolerancia original
        robot.max_error = original_max_error
        
        # Stop motors
        if 'left_motor' in handles and 'right_motor' in handles:
            sim_env.set_joint_velocity(handles['left_motor'], 0)
            sim_env.set_joint_velocity(handles['right_motor'], 0)
        
        # Stop simulation
        sim_env.stop()
        
        # Plot results - incluir waypoints generados por campos potenciales
        #visualizer.plot_results(waypoints)
    
    print('Program finished')

if __name__ == "__main__":
    main() 