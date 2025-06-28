# Código IPD482 - Robótica Móvil

Este repositorio contiene el material y los códigos desarrollados para el curso **IPD482 - Robótica Móvil** de la **Universidad Técnica Federico Santa María**. 

## Contenido del Repositorio

El repositorio está organizado en carpetas que corresponden a las distintas guías y ejercicios del curso:

- **`Guia1/`**: Contiene los ejercicios y simulaciones relacionados con los conceptos básicos de cinemática de robots móviles.
  - **`Imagenes y animaciones/`**: Diagramas, resultados de simulaciones y animaciones que acompañana a los modelos obtenidos.
  - **`Pregunta2/`**: Código para simular y animar la trayectoria de un robot omnidireccional siguiendo una línea recta mientras gira.
  - **`Pregunta3/`**: Código para simular y animar la cinemática de un N-trailer de 2 trailers pasivos.
  - **`Pregunta4/`**: Código para simular y animar la dinámica de un robot skid steer cuyas ruedas están sujetas a roce.
  
- **`Guia2/`**: Contiene los ejercicios relacionados con la simulación de un sistema G1T tanto en Coppelia como en Python.
  - **`ControlG1T/`**: Control hecho en el sistema G1T para seguir el camino trazado por campos potenciales y RTT.
  - **`PathPlanning/`**: Código para generar trayectorias utilizando RTT y Campos potenciales.
  - **`TestsModeloDin/`**: Implementaciones del modelo dinámico del sistema G1T.
  - **`demo/`**: Videos de demostración.
  - **`IPD482_guia2_2025_1.pdf`**: Archivo con las preguntas desarrolladas durante este trabajo.
  
- **`Guia3/`**: Contiene los ejercicios relacionados con la simulación de un sistema G1T aplicando SLAM.
  - **`components/`**: Scripts con distintas funciones y objetos desarrollados para distintas tareas.
  - **`coppelia/`**: Escenario de CoppeliaSim con el robot, entorno y sensores.
  - **`outputs/`**: Salidas obtenidas del programa, como imágenes y archivos `.json` con los datos obtenidos.
  

## Requisitos

Para ejecutar los códigos de la guía 1 se utilizó MATLAB Online, versión R2024b Update 5

En guía 2 y guía 3 se utilizó Python y CoppeliaSim
