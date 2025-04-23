import pygame
import math

# Configuración inicial
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Simulación Robot Omnidireccional con Trayectoria")
clock = pygame.time.Clock()

# Propiedades del robot
robot_radius = 50
wheel_radius = 15
num_wheels = 5
robot_position = [100, height // 2]  # Posición inicial [x, y]
robot_angle = 0  # Ángulo inicial de rotación
max_speed = 2  # Velocidad máxima del robot
rotation_speed = 0.05  # Velocidad de rotación constante

# Trayectoria (lista de puntos a seguir)
path = [[200, 300], [400, 400], [600, 200], [700, 500]]  # Coordenadas de los puntos
current_target_index = 0

# Función para calcular la velocidad usando cinemática
def calculate_velocity(current_pos, target_pos):
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    distance = math.sqrt(dx**2 + dy**2)

    # Calcular velocidades lineales y angulares
    if distance > 0.1:  # Umbral para evitar oscilaciones
        linear_velocity_x = (dx / distance) * max_speed
        linear_velocity_y = (dy / distance) * max_speed
    else:
        linear_velocity_x = 0
        linear_velocity_y = 0

    return linear_velocity_x, linear_velocity_y

# Función para dibujar el robot
def draw_robot(x, y, angle):
    # Dibujar el cuerpo principal
    pygame.draw.circle(screen, (0, 0, 255), (int(x), int(y)), robot_radius)

    # Dibujar las ruedas
    for i in range(num_wheels):
        wheel_angle = angle + (2 * math.pi / num_wheels) * i
        wheel_x = x + math.cos(wheel_angle) * robot_radius
        wheel_y = y + math.sin(wheel_angle) * robot_radius
        pygame.draw.circle(screen, (255, 0, 0), (int(wheel_x), int(wheel_y)), wheel_radius)

# Bucle principal
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Verificar si hay más puntos en la trayectoria
    if current_target_index < len(path):
        target_pos = path[current_target_index]
        # Calcular velocidades
        velocity_x, velocity_y = calculate_velocity(robot_position, target_pos)

        # Actualizar posición del robot
        robot_position[0] += velocity_x
        robot_position[1] += velocity_y

        # Chequear si se alcanzó el punto objetivo
        distance_to_target = math.sqrt(
            (target_pos[0] - robot_position[0])**2 +
            (target_pos[1] - robot_position[1])**2
        )
        if distance_to_target < 5:  # Umbral para cambiar al siguiente punto
            current_target_index += 1
    else:
        # Detener el movimiento una vez que se alcanzaron todos los puntos
        velocity_x, velocity_y = 0, 0

    # Actualizar el ángulo de rotación
    robot_angle += rotation_speed

    # Dibujar en pantalla
    screen.fill((255, 255, 255))  # Fondo blanco
    draw_robot(robot_position[0], robot_position[1], robot_angle)
    pygame.display.flip()

    # Control de fotogramas por segundo
    clock.tick(60)

pygame.quit()