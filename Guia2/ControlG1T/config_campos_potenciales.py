"""
Configuración para el algoritmo de campos potenciales
"""

def get_config():
    """
    Retorna la configuración para campos potenciales.
    """
    # Obtener obstáculos del archivo principal
    from campos_potenciales import ROBOT_WIDTH, ROBOT_LENGTH
    
    config = {
        # Parámetros de inicio y objetivo
        'start_x': -12.0,
        'start_y': -12.0,
        'goal_x': 9.0,
        'goal_y': 7.0,
        'grid_size': 0.5,
        
        # Dimensiones del robot (rectangular)
        'robot_width': ROBOT_WIDTH,    # ancho del tractor + trailer [m]
        'robot_length': ROBOT_LENGTH,  # largo total del tractor + trailer [m]
        
        # Para compatibilidad con código anterior que usa radio
        'robot_radius': max(ROBOT_WIDTH, ROBOT_LENGTH) / 2.0,
        
        # Configuración de waypoints
        'waypoint_height': 0.0,
        'max_waypoints': 50,  # Limitar waypoints para mejor control
        'use_dynamic_start': True,  # Usar posición inicial real del robot
        
        # Configuración de visualización
        'show_animation': False,  # Desactivar para simulación
        
        # Configuración de respaldo (en caso de error)
        'fallback_radius': 5.0,
        'fallback_segments': 12,
        
        # Obstáculos del entorno (copiados de campos_potenciales.py)
        'obstacle_x': [
            -10.6006, -8.3, -8.4, -8.25, -8.3506, -7.450, -3.450,
            -1.125-2.5, -1.125, -1.125+2.5,
            -1.075-2.5, -1.075, -1.075+2.5,
            -1.000-2.5, -1.000, -1.000+2.5,
            0.500, 
            6.700, 6.700, 6.700,
            6.725, 6.725, 6.725,
            5.9494, 10.1994, 10.2744
        ],
        
        'obstacle_y': [
            -9.99567, -9.60222, -6.1022, -2.3522, 0.46433, 6.12278, 6.172278,
            -9.725, -9.725, -9.725,
            -4.550, -4.550, -4.550,
            0.650, 0.650, 0.650,
            6.12278,
            -8.300-2.5, -8.300, -8.300+2.5,
            0.675-2.5, 0.675, 0.675+2.5,
            7.38933, 4.56443, 10.11433
        ]
    }
    
    return config

def print_config():
    """
    Imprime la configuración actual.
    """
    config = get_config()
    print("\n" + "="*60)
    print("CONFIGURACIÓN DE CAMPOS POTENCIALES")
    print("="*60)
    print(f"Posición inicial: ({config['start_x']}, {config['start_y']})")
    print(f"Objetivo: ({config['goal_x']}, {config['goal_y']})")
    print(f"Resolución de grilla: {config['grid_size']} m")
    print(f"Dimensiones del robot: {config['robot_width']} x {config['robot_length']} m")
    print(f"Número de obstáculos: {len(config['obstacle_x'])}")
    print(f"Máximo waypoints: {config['max_waypoints']}")
    print(f"Altura de waypoints: {config['waypoint_height']} m")
    print(f"Animación activada: {config['show_animation']}")
    print(f"Posición inicial dinámica: {config['use_dynamic_start']}")
    print("="*60) 