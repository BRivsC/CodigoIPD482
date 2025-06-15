# Sistema de Control por Torque para Tractor-Trailer en CoppeliaSim

## Descripción

Este proyecto implementa un sistema de control por torque para un tractor-trailer que sigue rutas generadas por campos potenciales. El sistema integra:

- **Modelo dinámico completo** del tractor-trailer con efectos de fricción LuGre
- **Planificación de rutas** usando campos potenciales
- **Control por torque** basado en el modelo dinámico
- **Estimación de estados** con filtro de Kalman simplificado
- **Visualización avanzada** de resultados

## Archivos del Sistema

### 1. `control_torque_coppelia.py` - Controlador Básico
**Características:**
- Control PD básico con compensación feedforward
- Interface simple con CoppeliaSim
- Estimación de velocidades simplificada
- Seguimiento de waypoints con tolerancia

**Uso:**
```bash
python control_torque_coppelia.py
```

### 2. `control_dinamico_avanzado.py` - Controlador Avanzado ⭐ (RECOMENDADO)
**Características:**
- Control PID con compensación del modelo dinámico
- Estimador de estados con filtro de Kalman
- Predicción de trayectorias con lookahead
- Visualización comprehensiva de métricas
- Manejo robusto de errores

**Uso:**
```bash
python control_dinamico_avanzado.py
```

## Dependencias

### Librerías Python
```bash
pip install numpy casadi matplotlib
```

### CoppeliaSim
- Versión 4.0 o superior
- Módulo Python `sim` configurado
- Escena con tractor-trailer (Pioneer P3DX)

## Configuración del Entorno

### 1. Configuración de CoppeliaSim
- Abrir CoppeliaSim
- Cargar escena con tractor-trailer
- Verificar que los handles de objetos coincidan con los del código:
  ```
  /PioneerP3DX/leftMotor
  /PioneerP3DX/rightMotor
  /PioneerP3DX/GPS
  /PioneerP3DX2/GPS2
  ```

### 2. Configuración de la Simulación
- Tiempo de simulación: modo real-time o acelerado
- Frecuencia de control: 20 Hz (dt = 0.05s)
- Puerto de comunicación: 19999 (default)

## Parámetros del Sistema

### Parámetros Físicos (de `dinamica.py`)
```python
P = {
    'm1': 4.0,    # masa tractor [kg]
    'm2': 4.0,    # masa trailer [kg]
    'mw': 0.2,    # masa rueda [kg]
    'I1': 0.4,    # inercia tractor [kg⋅m²]
    'I2': 0.4,    # inercia trailer [kg⋅m²]
    'b': 0.2,     # medio-eje [m]
    'r': 0.1,     # radio rueda [m]
    'L1': 1.0,    # longitud tractor [m]
    'L2': 1.0,    # longitud trailer [m]
    'mu': -0.555  # coeficiente de steering pasivo
}
```

### Parámetros de Control
```python
# Ganancias PID
kp_v = 1.5      # Proporcional velocidad
ki_v = 0.2      # Integral velocidad
kd_v = 0.4      # Derivativo velocidad
kp_w = 2.0      # Proporcional angular
ki_w = 0.1      # Integral angular
kd_w = 0.3      # Derivativo angular

# Límites
max_torque = 1.5        # [N⋅m]
max_velocity = 1.2      # [m/s]
max_angular_vel = 1.0   # [rad/s]
```

### Tolerancias
```python
waypoint_tolerance = 1.2  # Distancia para considerar waypoint alcanzado [m]
goal_tolerance = 1.5      # Distancia para considerar objetivo alcanzado [m]
```

## Arquitectura del Sistema

```
┌─────────────────────┐
│   Campos            │
│   Potenciales       │ → Ruta (path_x, path_y)
└─────────────────────┘
           ↓
┌─────────────────────┐
│   Controlador       │
│   de Torque         │ → Torques (τᵣ, τₗ)
└─────────────────────┘
           ↓
┌─────────────────────┐
│   CoppeliaSim       │
│   (Simulación)      │ → Estados (x, y, θ₁, θ₂)
└─────────────────────┘
           ↓
┌─────────────────────┐
│   Estimador         │
│   de Estados        │ → Estado estimado X̂
└─────────────────────┘
```

## Flujo de Control

### 1. Inicialización
- Conexión con CoppeliaSim
- Obtención de handles de objetos
- Configuración de sensores GPS
- Generación de ruta con campos potenciales

### 2. Bucle de Control (20 Hz)
```python
while not path_completed:
    # 1. Predicción de estados
    state_estimator.predict(tau_r, tau_l, t)
    
    # 2. Actualización con mediciones
    state_estimator.update(gps_tractor, gps_trailer)
    
    # 3. Cálculo de velocidades deseadas
    desired_v, desired_w = compute_desired_trajectory()
    
    # 4. Control PID + compensación
    tau_r, tau_l = compute_model_based_torques(desired_v, desired_w)
    
    # 5. Aplicación de torques
    sim_env.set_joint_torque(left_motor, tau_l)
    sim_env.set_joint_torque(right_motor, tau_r)
```

### 3. Finalización
- Detención de motores
- Generación de gráficos de resultados
- Análisis de métricas de performance

## Métricas de Evaluación

### Precisión de Seguimiento
- **Error promedio de tracking**: Distancia promedio a la ruta deseada
- **Error máximo de tracking**: Mayor desviación de la ruta
- **Error final**: Distancia al objetivo final

### Eficiencia Energética
- **Torque RMS**: Esfuerzo de control promedio
- **Suavidad**: Variaciones en los torques aplicados

### Performance Temporal
- **Tiempo de convergencia**: Tiempo para alcanzar el objetivo
- **Estabilidad**: Ausencia de oscilaciones

## Configuración de Parámetros

### Campos Potenciales
```python
# Configuración en campos_potenciales.py
configure_debug_parameters(
    debug_mode=False,        # Activar/desactivar debug
    max_iterations=3000,     # Máximo de iteraciones
    goal_tolerance=1.0       # Tolerancia del objetivo [m]
)
```

### Ajuste de Ganancias PID
Para ajustar el comportamiento del controlador:

**Para mejor seguimiento de trayectoria:**
- Aumentar `kp_v` y `kp_w`
- Cuidado con overshoot

**Para suavidad:**
- Aumentar `kd_v` y `kd_w`
- Reducir oscilaciones

**Para eliminación de error estacionario:**
- Aumentar `ki_v` y `ki_w`
- Implementar anti-windup

## Solución de Problemas

### Problema: Robot no se mueve
**Posibles causas:**
- Handles de motores incorrectos
- Torques muy bajos
- Fricción excesiva en simulación

**Solución:**
- Verificar handles con `print(handles)`
- Aumentar `max_torque`
- Ajustar propiedades de fricción en CoppeliaSim

### Problema: Oscilaciones en el control
**Causas:**
- Ganancias PID muy altas
- Ruido en sensores
- Frecuencia de control muy baja

**Solución:**
- Reducir ganancias proporcionales
- Implementar filtrado de señales
- Aumentar frecuencia de control

### Problema: No alcanza el objetivo
**Causas:**
- Tolerancias muy estrictas
- Obstáculos no considerados
- Límites de velocidad muy bajos

**Solución:**
- Ajustar tolerancias
- Verificar ruta libre de obstáculos
- Aumentar límites de velocidad

## Extensiones Futuras

### 1. Control Predictivo (MPC)
- Optimización con horizonte de predicción
- Manejo de restricciones
- Mejor performance en curvas

### 2. Estimación Avanzada
- EKF completo para estimación no lineal
- Sensor fusion con IMU
- Estimación de parámetros del modelo

### 3. Adaptación Online
- Identificación de parámetros
- Adaptación a terrenos variables
- Aprendizaje por refuerzo

## Referencias

- Modelo dinámico basado en literatura de tractor-trailer
- Control PID clásico con compensación feedforward
- Campos potenciales para planificación de rutas
- Filtro de Kalman para estimación de estados

## Licencia

Este código es parte de un proyecto académico. Usar libremente para propósitos educativos. 