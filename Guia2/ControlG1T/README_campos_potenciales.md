# Control de Robot con Campos Potenciales en CoppeliaSim

Este proyecto implementa un sistema de navegación autónoma para un robot tractor-trailer usando **campos potenciales** para la planificación de trayectorias y **control PID** para el seguimiento de waypoints en CoppeliaSim.

## 🎯 Características Principales

- **Planificación de Trayectorias**: Algoritmo de campos potenciales con escape de mínimos locales
- **Control Robusto**: Controlador PID con suavizado de velocidad y límites de seguridad
- **Robot Rectangular**: Modelado preciso del conjunto tractor-trailer
- **Visualización**: Gráficos en tiempo real de la trayectoria seguida
- **Configuración Flexible**: Parámetros ajustables via archivo de configuración

## 📁 Estructura del Proyecto

```
tarea2/
├── campos_potenciales.py      # Algoritmo de campos potenciales
├── config_campos_potenciales.py  # Configuración del sistema
├── script_coppelia.py         # Script principal para CoppeliaSim
├── test_campos_potenciales.py # Script de pruebas
├── components/
│   ├── simulation.py          # Interfaz con CoppeliaSim
│   ├── sensors.py             # Sensores GPS
│   ├── robot_controller.py    # Controlador PID del robot
│   ├── visualization.py       # Visualización de resultados
│   ├── controllers.py         # Controladores PID base
│   └── utils.py              # Utilidades matemáticas
└── README_campos_potenciales.md
```

## 🚀 Instalación y Ejecución

### Requisitos

```bash
pip install numpy matplotlib coppeliasim-zmqremoteapi-client
```

### Prueba del Algoritmo (sin CoppeliaSim)

```bash
python test_campos_potenciales.py
```

### Ejecución en CoppeliaSim

1. Abrir CoppeliaSim con la escena que contiene el robot tractor-trailer
2. Ejecutar el script principal:

```bash
python script_coppelia.py
```

## ⚙️ Configuración

### Archivo `config_campos_potenciales.py`

```python
config = {
    # Parámetros de inicio y objetivo
    'start_x': -12.0,
    'start_y': -12.0,
    'goal_x': 9.0,
    'goal_y': 7.0,
    'grid_size': 0.5,           # Resolución del mapa [m]
    
    # Dimensiones del robot
    'robot_width': 0.5,         # Ancho total [m]
    'robot_length': 1.5,        # Largo total [m]
    
    # Control de waypoints
    'max_waypoints': 50,        # Límite de waypoints
    'waypoint_height': 0.0,     # Altura Z de waypoints
    'use_dynamic_start': True,  # Usar posición inicial real
    
    # Visualización
    'show_animation': False,    # Activar gráficos durante planning
}
```

### Parámetros de Campos Potenciales (`campos_potenciales.py`)

```python
# Ganancias del potencial
KP = 5.0        # Ganancia atractiva
ETA = 100.0     # Ganancia repulsiva
AREA_WIDTH = 30.0  # Área de cálculo [m]

# Detección de oscilaciones
OSCILLATIONS_DETECTION_LENGTH = 5
```

### Parámetros del Controlador (`robot_controller.py`)

```python
# Ganancias PID
self.kp = 4.0    # Proporcional
self.ki = 0.02   # Integral
self.kd = 0.1    # Derivativo

# Límites de control
self.max_velocity = 2.5   # Velocidad máxima [m/s]
self.max_angular = 1.5    # Velocidad angular máxima [rad/s]
self.max_error = 0.1      # Tolerancia de waypoint [m]
```

## 🧮 Algoritmo de Campos Potenciales

### 1. Potencial Atractivo
```
U_att(x,y) = 0.5 * KP * ||p - p_goal||²
```

### 2. Potencial Repulsivo (Robot Rectangular)
```
U_rep(x,y) = 0.5 * ETA * (1/d - 1/r_influence)²  si d ≤ r_influence
           = 0                                    si d > r_influence
```

Donde `d` es la distancia mínima desde el rectángulo del robot al obstáculo más cercano.

### 3. Escape de Mínimos Locales

El algoritmo implementa varias estrategias:

- **Detección de oscilaciones**: Monitorea posiciones repetidas
- **Movimiento aleatorio dirigido**: Preferencia hacia el objetivo
- **Expansión del modelo de movimiento**: Saltos más grandes cuando es necesario

### 4. Validación de Trayectoria

- **Continuidad**: Verifica que no haya saltos grandes entre waypoints
- **Optimización**: Elimina waypoints muy cercanos para mejor control
- **Métricas**: Calcula eficiencia y estadísticas de la ruta

## 🎮 Control del Robot

### Arquitectura de Control

```
Campos Potenciales → Waypoints → Controlador PID → Motores
     ↑                             ↓
Obstáculos                    Sensores GPS
```

### Controlador PID

- **Control de Distancia**: PID para velocidad lineal basado en distancia al waypoint
- **Control Angular**: PID para velocidad angular basado en error de orientación
- **Suavizado**: Filtro de velocidad para evitar cambios abruptos
- **Límites Adaptativos**: Reduce velocidad en curvas cerradas

### Seguimiento de Trayectoria

1. **Navegación por Waypoints**: El robot navega secuencialmente a cada punto
2. **Tolerancias Adaptativas**: Mayor tolerancia para waypoints intermedios, menor para el objetivo final
3. **Timeout**: Evita quedarse atascado en un waypoint
4. **Estadísticas**: Reporta tasa de éxito y métricas de performance

## 📊 Métricas y Visualización

### Métricas Calculadas

- **Distancia Total**: Longitud de la trayectoria generada
- **Eficiencia**: Relación entre distancia directa y trayectoria real
- **Tasa de Éxito**: Porcentaje de waypoints alcanzados
- **Continuidad**: Máximo salto entre waypoints consecutivos

### Visualización

- **Trayectoria en 2D**: Muestra ruta del robot y trailer
- **Posición vs Tiempo**: Evolución temporal de coordenadas X e Y
- **Waypoints**: Marcadores de puntos objetivo
- **Obstáculos**: Visualización del entorno

## 🔧 Solución de Problemas

### Errores Comunes

1. **"Trayectoria muy corta"**
   - Verificar que start ≠ goal
   - Ajustar parámetros KP y ETA
   - Revisar posición de obstáculos

2. **"Saltos grandes detectados"**
   - Reducir `grid_size` para mayor resolución
   - Ajustar `max_jump` en validación

3. **"Robot no alcanza waypoints"**
   - Aumentar `max_error` tolerance
   - Verificar ganancias PID
   - Revisar límites de velocidad

### Ajuste de Parámetros

**Para mejor evitación de obstáculos:**
- Aumentar `ETA` (potencial repulsivo)
- Reducir `grid_size` (mayor precisión)

**Para trayectorias más suaves:**
- Aumentar `KP` (potencial atractivo)
- Usar `optimize_waypoints()` con mayor `min_distance`

**Para mejor seguimiento:**
- Ajustar ganancias PID del controlador
- Modificar límites de velocidad
- Cambiar factor de suavizado

## 🧪 Pruebas y Validación

### Script de Pruebas

```bash
python test_campos_potenciales.py
```

Este script ejecuta:
- Prueba de generación de trayectoria básica
- Validación de diferentes configuraciones
- Análisis de métricas de performance
- Generación de gráficos de validación

### Métricas de Validación

- ✅ **Éxito**: Trayectoria generada correctamente
- 📍 **Puntos**: Número de waypoints en la trayectoria
- 📏 **Distancia**: Longitud total del camino
- ⚡ **Eficiencia**: Optimización respecto a línea directa
- 🦘 **Continuidad**: Saltos máximos entre puntos

## 📝 Desarrollo y Contribución

### Estructura del Código

- **Principios SOLID**: Separación de responsabilidades
- **Documentación**: Funciones y clases documentadas
- **Manejo de Errores**: Validación robusta y recuperación
- **Testing**: Scripts de prueba automatizados

### Extensiones Posibles

1. **Algoritmos Avanzados**: A*, RRT, D* Lite
2. **Control Predictivo**: Model Predictive Control (MPC)
3. **Múltiples Robots**: Coordinación de flotas
4. **Aprendizaje**: Reinforcement Learning para optimización

## 📚 Referencias

- Khatib, O. (1986). "Real-time obstacle avoidance for manipulators and mobile robots"
- LaValle, S. M. (2006). "Planning Algorithms"
- Siegwart, R. (2011). "Introduction to Autonomous Mobile Robots"

---

**Desarrollado para el curso de Robótica Móvil**  
*Sistema de navegación con campos potenciales y control PID* 