# Contexto del Proyecto - Sistema de Navegación con Campos Potenciales

## Descripción General

Este proyecto implementa un sistema de navegación autónoma para un robot con tráiler usando **campos potenciales** para la planificación de trayectorias y control **PID** para el seguimiento de waypoints en el simulador **CoppeliaSim**.

## Arquitectura del Sistema

### Componentes Principales

1. **Planificación de Trayectoria (`campos_potenciales.py`)**
   - Algoritmo de campos potenciales para evitar obstáculos
   - Generación de waypoints desde posición inicial hasta objetivo
   - Consideración de las dimensiones rectangulares del robot + tráiler

2. **Control del Robot (`components/robot_controller.py`)**
   - Control PID dual para distancia y orientación
   - Navegación punto a punto con tolerancias configurables
   - Suavizado de velocidades para mejor estabilidad

3. **Sensores (`components/sensors.py`)**
   - GPS para posicionamiento del robot principal y tráiler
   - Obtención de orientación y posiciones en tiempo real

4. **Simulación (`components/simulation.py`)**
   - Interfaz con CoppeliaSim
   - Manejo de handles de objetos
   - Control de motores y articulaciones

5. **Visualización Avanzada (`components/visualization.py`)** ⭐ **NUEVA CARACTERÍSTICA**
   - Múltiples ventanas de análisis de desempeño
   - Comparación de trayectorias planificada vs real
   - Métricas detalladas de error y rendimiento

## Mejoras Implementadas

### Sistema de Visualización Mejorado

Se ha expandido significativamente el sistema de visualización para proporcionar un análisis completo del desempeño:

#### 1. **Gráfico de Comparación de Trayectorias** (`*_comparison.png`)
- **Trayectoria Planificada**: Ruta generada por campos potenciales (línea roja discontinua)
- **Trayectoria Real**: Ruta seguida por el robot (línea azul sólida)
- **Lecturas GPS**: Si difieren de la posición real (línea verde punteada)
- **Trayectoria del Tráiler**: Si está disponible (línea magenta)
- **Métricas en tiempo real**: Error promedio, máximo y RMS mostrados en el gráfico

#### 2. **Análisis de Desempeño** (`*_performance.png`)
Ventana con 4 sub-gráficos:

- **Errores de Seguimiento vs Tiempo**: 
  - Evolución temporal del error de distancia
  - Líneas de referencia para error promedio y desviación estándar
  
- **Perfil de Velocidad**:
  - Velocidad lineal a lo largo del tiempo
  - Marcadores para cambios de waypoint
  - Línea de velocidad promedio
  
- **Velocidad Angular**:
  - Cambios en la orientación del robot
  - Útil para detectar maniobras bruscas
  
- **Error Transversal (Cross-track)**:
  - Desviación lateral respecto a la trayectoria planificada
  - Área sombreada para mostrar variabilidad

#### 3. **Estadísticas Resumidas** (`*_statistics.png`)
Ventana con análisis estadístico completo:

- **Histograma de Errores**: Distribución de errores de seguimiento
- **Histograma de Velocidades**: Distribución de velocidades del robot
- **Tabla de Métricas**: Resumen numérico de todos los indicadores
- **Análisis de Waypoints**: Comparación de tiempos estimados vs reales

#### 4. **Resumen en Consola**
Sistema de calificación automática del desempeño:
- 🎯 **EXCELENTE**: Error promedio < 0.5m
- ✅ **BUENO**: Error promedio < 1.0m  
- ⚠️ **REGULAR**: Error promedio < 2.0m
- ❌ **NECESITA MEJORAS**: Error promedio ≥ 2.0m

### Métricas Calculadas

#### Errores de Seguimiento
- **Error Promedio**: Distancia media entre trayectoria planificada y real
- **Error Máximo**: Mayor desviación registrada
- **Error RMS**: Raíz cuadrada del error cuadrático medio
- **Desviación Estándar**: Variabilidad del error
- **Error Transversal**: Desviación lateral firmada respecto al camino

#### Rendimiento del Robot
- **Velocidad Promedio/Máxima**: Análisis de la dinámica del robot
- **Velocidad Angular**: Análisis de maniobras y giros
- **Tiempo Total**: Duración de la misión
- **Distancia Total**: Longitud real del recorrido
- **Eficiencia de Waypoints**: Porcentaje de waypoints alcanzados exitosamente

## Configuración

El sistema usa `config_campos_potenciales.py` para configurar:

- Posiciones de inicio y objetivo
- Dimensiones del robot (rectangular: ancho x largo)
- Posiciones de obstáculos
- Parámetros de visualización y control
- Límites de waypoints y tolerancias

## Uso

### Ejecución Principal
```bash
python script_coppelia.py
```

### Pruebas Sin Simulador
```bash
python test_campos_potenciales.py
```

## Archivos Generados

Después de cada ejecución se crean múltiples archivos de análisis:

1. `trayectoria_robot.png` - Gráfico original (compatibilidad)
2. `trayectoria_robot_comparison.png` - Comparación de trayectorias
3. `trayectoria_robot_performance.png` - Análisis de desempeño detallado
4. `trayectoria_robot_statistics.png` - Estadísticas resumidas

## Principios de Diseño Aplicados

### Seguimiento de Reglas de Calidad

- **DRY**: Funciones de análisis reutilizables y modulares
- **SRP**: Cada método tiene una responsabilidad específica
- **Error Handling**: Manejo robusto de casos edge y datos faltantes
- **Naming**: Nombres descriptivos y auto-documentados
- **Testing**: Validación de continuidad y integridad de datos

### Características de Robustez

- **Validación de Datos**: Verificación de dimensiones y datos válidos
- **Fallbacks**: Gráficos alternativos cuando faltan datos
- **Exception Handling**: Manejo gracioso de errores de plotting
- **Backward Compatibility**: Mantiene compatibilidad con código existente

## Próximas Mejoras Sugeridas

1. **Métricas de Energía**: Análisis del consumo energético del robot
2. **Predicción de Trayectoria**: Comparación con modelos predictivos
3. **Análisis de Obstáculos**: Efectividad de la evasión de obstáculos
4. **Optimización de Parámetros**: Sugerencias automáticas de mejora
5. **Export de Datos**: Exportación a formatos CSV/JSON para análisis externos

---

**Última actualización**: Implementación de sistema de visualización avanzado con análisis de desempeño multi-ventana y métricas detalladas. 