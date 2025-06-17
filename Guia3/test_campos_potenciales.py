#!/usr/bin/env python3
"""
Script de prueba para validar la generación de trayectorias con campos potenciales.
Permite probar el algoritmo sin necesidad de CoppeliaSim.
"""

import matplotlib.pyplot as plt
from campos_potenciales import potential_field_planning, ROBOT_WIDTH, ROBOT_LENGTH
from config_campos_potenciales import get_config, print_config
import numpy as np

def test_potential_field_planning():
    """Prueba la generación de trayectorias con campos potenciales."""
    
    print("="*60)
    print("PRUEBA DE CAMPOS POTENCIALES")
    print("="*60)
    
    # Cargar configuración
    config = get_config()
    print_config()
    
    # Parámetros de la prueba
    start_x = config['start_x']
    start_y = config['start_y']
    goal_x = config['goal_x']
    goal_y = config['goal_y']
    grid_size = config['grid_size']
    robot_width = config['robot_width']
    robot_length = config['robot_length']
    
    obstacle_x = config['obstacle_x']
    obstacle_y = config['obstacle_y']
    
    print(f"\n🎯 Probando ruta desde ({start_x}, {start_y}) hasta ({goal_x}, {goal_y})")
    print(f"📐 Dimensiones del robot: {robot_width} x {robot_length} m")
    print(f"🚧 Número de obstáculos: {len(obstacle_x)}")
    
    try:
        # Activar animación para la prueba
        import campos_potenciales
        campos_potenciales.show_animation = True
        
        # Generar trayectoria
        print("\n🔄 Generando trayectoria...")
        path_x, path_y = potential_field_planning(
            start_x, start_y, goal_x, goal_y,
            obstacle_x, obstacle_y, grid_size, robot_width, robot_length
        )
        
        # Validar resultados
        if len(path_x) < 2 or len(path_y) < 2:
            print("❌ Error: Trayectoria muy corta")
            return False
        
        # Calcular métricas
        total_distance = 0
        for i in range(1, len(path_x)):
            dx = path_x[i] - path_x[i-1]
            dy = path_y[i] - path_y[i-1]
            total_distance += np.sqrt(dx*dx + dy*dy)
        
        direct_distance = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        efficiency = direct_distance / total_distance * 100
        
        print("\n📊 RESULTADOS:")
        print(f"   ✅ Trayectoria generada exitosamente")
        print(f"   📍 Puntos en la trayectoria: {len(path_x)}")
        print(f"   📏 Distancia total: {total_distance:.2f} m")
        print(f"   📐 Distancia directa: {direct_distance:.2f} m")
        print(f"   ⚡ Eficiencia: {efficiency:.1f}%")
        
        # Validar continuidad
        max_jump = 0
        for i in range(1, len(path_x)):
            dx = path_x[i] - path_x[i-1]
            dy = path_y[i] - path_y[i-1]
            jump = np.sqrt(dx*dx + dy*dy)
            max_jump = max(max_jump, jump)
        
        print(f"   🦘 Salto máximo entre puntos: {max_jump:.2f} m")
        
        if max_jump > 3.0:
            print("   ⚠️  Advertencia: Saltos grandes detectados")
        else:
            print("   ✅ Continuidad de trayectoria: OK")
        
        # Mostrar primeros y últimos puntos
        print(f"\n📍 Primeros 3 puntos:")
        for i in range(min(3, len(path_x))):
            print(f"     {i+1}: ({path_x[i]:.2f}, {path_y[i]:.2f})")
        
        print(f"📍 Últimos 3 puntos:")
        start_idx = max(0, len(path_x) - 3)
        for i in range(start_idx, len(path_x)):
            print(f"     {i+1}: ({path_x[i]:.2f}, {path_y[i]:.2f})")
        
        return True
        
    except Exception as e:
        print(f"❌ Error al generar trayectoria: {e}")
        import traceback
        traceback.print_exc()
        return False

def plot_trajectory_summary(path_x, path_y, obstacle_x, obstacle_y, config):
    """Genera un gráfico resumen de la trayectoria."""
    
    plt.figure(figsize=(12, 10))
    
    # Plot obstáculos
    plt.scatter(obstacle_x, obstacle_y, c='red', s=100, marker='x', 
                linewidths=3, label='Obstáculos', alpha=0.8)
    
    # Plot trayectoria
    plt.plot(path_x, path_y, 'b-', linewidth=2, label='Trayectoria generada')
    plt.plot(path_x[0], path_y[0], 'go', markersize=10, label='Inicio')
    plt.plot(path_x[-1], path_y[-1], 'ro', markersize=10, label='Objetivo')
    
    # Plot línea directa
    plt.plot([path_x[0], path_x[-1]], [path_y[0], path_y[-1]], 
             'k--', alpha=0.5, label='Línea directa')
    
    # Configuración del gráfico
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trayectoria Generada por Campos Potenciales')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Añadir información
    plt.text(0.02, 0.98, f'Puntos: {len(path_x)}', transform=plt.gca().transAxes, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig('trayectoria_campos_potenciales.png', dpi=150, bbox_inches='tight')
    print(f"\n💾 Gráfico guardado como: trayectoria_campos_potenciales.png")

def test_configuration_variations():
    """Prueba diferentes configuraciones para evaluar robustez."""
    
    print("\n" + "="*60)
    print("PRUEBA DE VARIACIONES DE CONFIGURACIÓN")
    print("="*60)
    
    config = get_config()
    
    variations = [
        {"name": "Configuración Original", "changes": {}},
        {"name": "Inicio Alternativo", "changes": {"start_x": -10.0, "start_y": -10.0}},
        {"name": "Objetivo Alternativo", "changes": {"goal_x": 8.0, "goal_y": 5.0}},
        {"name": "Grid Más Fino", "changes": {"grid_size": 0.3}},
        {"name": "Grid Más Grueso", "changes": {"grid_size": 0.8}},
    ]
    
    results = []
    
    for variation in variations:
        print(f"\n🧪 Probando: {variation['name']}")
        
        # Aplicar cambios
        test_config = config.copy()
        test_config.update(variation['changes'])
        
        try:
            # Desactivar animación para pruebas rápidas
            import campos_potenciales
            campos_potenciales.show_animation = False
            
            path_x, path_y = potential_field_planning(
                test_config['start_x'], test_config['start_y'],
                test_config['goal_x'], test_config['goal_y'],
                test_config['obstacle_x'], test_config['obstacle_y'],
                test_config['grid_size'], test_config['robot_width'], test_config['robot_length']
            )
            
            if len(path_x) >= 2:
                total_distance = sum(np.sqrt((path_x[i] - path_x[i-1])**2 + (path_y[i] - path_y[i-1])**2) 
                                   for i in range(1, len(path_x)))
                results.append({
                    "name": variation['name'],
                    "success": True,
                    "points": len(path_x),
                    "distance": total_distance
                })
                print(f"   ✅ Éxito - {len(path_x)} puntos, {total_distance:.2f}m")
            else:
                results.append({"name": variation['name'], "success": False})
                print(f"   ❌ Fallo - trayectoria muy corta")
                
        except Exception as e:
            results.append({"name": variation['name'], "success": False})
            print(f"   ❌ Error: {str(e)}")
    
    # Resumen de resultados
    print(f"\n📊 RESUMEN DE PRUEBAS:")
    successful = sum(1 for r in results if r.get('success', False))
    print(f"   Configuraciones exitosas: {successful}/{len(results)}")
    
    for result in results:
        if result.get('success', False):
            print(f"   ✅ {result['name']}: {result['points']} puntos, {result['distance']:.1f}m")
        else:
            print(f"   ❌ {result['name']}: Falló")

if __name__ == "__main__":
    print("🚀 Iniciando pruebas de campos potenciales...")
    
    # Prueba principal
    success = test_potential_field_planning()
    
    if success:
        print("\n🎉 ¡Prueba principal exitosa!")
        
        # Pruebas adicionales
        try:
            test_configuration_variations()
        except Exception as e:
            print(f"\n⚠️  Error en pruebas adicionales: {e}")
    else:
        print("\n❌ Prueba principal falló. Revisa la configuración.")
    
    print("\n" + "="*60)
    print("FIN DE PRUEBAS")
    print("="*60) 