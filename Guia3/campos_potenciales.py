from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import random

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 5  # Aumentado para ser menos estricto

# Robot rectangular dimensions (tractor + trailer)
ROBOT_WIDTH = 0.5   # ancho del tractor + trailer [m]
ROBOT_LENGTH = 1.50  # largo total del tractor + trailer [m]

show_animation = True


def calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, robot_width, robot_length, start_x, start_y):
    minx = min(min(obstacle_x), start_x, goal_x) - AREA_WIDTH / 2.0
    miny = min(min(obstacle_y), start_y, goal_y) - AREA_WIDTH / 2.0
    maxx = max(max(obstacle_x), start_x, goal_x) + AREA_WIDTH / 2.0
    maxy = max(max(obstacle_y), start_y, goal_y) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, goal_x, goal_y)
            uo = calc_repulsive_potential_rectangular(x, y, obstacle_x, obstacle_y, robot_width, robot_length)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, goal_x, goal_y):
    return 0.5 * KP * np.hypot(x - goal_x, y - goal_y)


def calc_repulsive_potential_rectangular(x, y, obstacle_x, obstacle_y, robot_width, robot_length):
    """
    Calcula el potencial repulsivo considerando que el robot es rectangular.
    El robot se modela como un rectángulo centrado en (x, y).
    """
    # Buscar el obstáculo más cercano
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(obstacle_x):
        d = np.hypot(x - obstacle_x[i], y - obstacle_y[i])
        if dmin >= d:
            dmin = d
            minid = i

    # Calcular la distancia mínima desde el rectángulo del robot al obstáculo más cercano
    dq = calc_min_distance_rectangle_to_point(x, y, robot_width, robot_length, 
                                              obstacle_x[minid], obstacle_y[minid])
    
    # Radio de influencia basado en las dimensiones del robot
    rr = max(robot_width, robot_length) / 2.0 + 1.0  # margen de seguridad adicional

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1
        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def calc_min_distance_rectangle_to_point(rect_x, rect_y, rect_width, rect_length, point_x, point_y):
    """
    Calcula la distancia mínima desde un rectángulo centrado en (rect_x, rect_y) 
    hasta un punto (point_x, point_y).
    """
    # Coordenadas de los bordes del rectángulo
    half_width = rect_width / 2.0
    half_length = rect_length / 2.0
    
    left = rect_x - half_width
    right = rect_x + half_width
    bottom = rect_y - half_length
    top = rect_y + half_length
    
    # Calcular la distancia mínima
    dx = max(left - point_x, 0, point_x - right)
    dy = max(bottom - point_y, 0, point_y - top)
    
    return np.hypot(dx, dy)


def get_motion_model():
    # dx, dy
    # all the 8 neighbouring cells to be checked
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # Contar cuántas veces aparece la posición actual en el historial reciente
    count = sum(1 for pos in previous_ids if pos == (ix, iy))
    
    # Si la posición actual aparece más de 2 veces en el historial, es oscilación
    return count > 2


def add_random_walk_motion(motion):
    """
    Añade movimientos aleatorios adicionales para escapar de mínimos locales
    """
    # Movimientos adicionales con mayor rango
    additional_motions = [
        [2, 0], [0, 2], [-2, 0], [0, -2],
        [2, 1], [1, 2], [-2, 1], [-1, 2],
        [2, -1], [1, -2], [-2, -1], [-1, -2]
    ]
    
    return motion + additional_motions


def escape_local_minimum(ix, iy, pmap, motion, minx, miny, reso, goal_x, goal_y):
    """
    Intenta escapar de un mínimo local aplicando una estrategia de movimiento aleatorio
    hacia la dirección general de la meta
    """
    print(f"Intentando escapar de mínimo local en ({ix}, {iy})")
    
    # Calcular dirección hacia la meta
    current_x = ix * reso + minx
    current_y = iy * reso + miny
    
    goal_direction_x = 1 if goal_x > current_x else -1
    goal_direction_y = 1 if goal_y > current_y else -1
    
    # Crear movimientos preferentes hacia la meta con algo de aleatoriedad
    preferred_motions = []
    
    # Movimientos hacia la meta
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            if dx == 0 and dy == 0:
                continue
            
            # Dar más peso a movimientos hacia la meta
            weight = 1
            if dx * goal_direction_x > 0:
                weight += 2
            if dy * goal_direction_y > 0:
                weight += 2
                
            for _ in range(weight):
                preferred_motions.append([dx, dy])
    
    # Mezclar aleatoriamente
    random.shuffle(preferred_motions)
    
    # Intentar cada movimiento preferente
    for move in preferred_motions[:10]:  # Probar los primeros 10
        inx = int(ix + move[0])
        iny = int(iy + move[1])
        
        if 0 <= inx < len(pmap) and 0 <= iny < len(pmap[0]):
            return inx, iny
    
    # Si no encuentra nada, usar movimiento aleatorio básico
    for _ in range(20):
        move = random.choice(motion)
        inx = int(ix + move[0])
        iny = int(iy + move[1])
        
        if 0 <= inx < len(pmap) and 0 <= iny < len(pmap[0]):
            return inx, iny
    
    # En último caso, quedarse en la misma posición
    return ix, iy


def potential_field_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, reso, robot_width, robot_length):
    # calc potential field
    pmap, minx, miny = calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, robot_width, robot_length, start_x, start_y)

    # search path
    d = np.hypot(start_x - goal_x, start_y - goal_y)
    ix = round((start_x - minx) / reso)
    iy = round((start_y - miny) / reso)
    gix = round((goal_x - minx) / reso)
    giy = round((goal_y - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "oy")
        plt.plot(gix, giy, "ok")
        plt.annotate("GOAL", xy=(gix + 2, giy + 2))
        plt.annotate("START", xy=(25, 22), color='yellow')
        plt.axis(False)
        
        # Dibujar el robot rectangular en la posición inicial
        draw_robot_rectangle(start_x, start_y, robot_width, robot_length, minx, miny, reso, 'yellow')
    
    path_x, path_y = [start_x], [start_y]
    motion = get_motion_model()
    previous_ids = deque()
    oscillation_count = 0
    max_iterations = 1000  # Límite de iteraciones para evitar bucles infinitos

    iteration = 0
    while d >= reso and iteration < max_iterations:
        iteration += 1
        
        # Detectar oscilaciones
        if oscillations_detection(previous_ids, ix, iy):
            oscillation_count += 1
            print(f"Oscillation detected at ({ix},{iy})! Count: {oscillation_count}")
            
            if oscillation_count >= 3:  # Después de 3 oscilaciones, usar escape
                ix, iy = escape_local_minimum(ix, iy, pmap, motion, minx, miny, reso, goal_x, goal_y)
                oscillation_count = 0
                previous_ids.clear()  # Limpiar historial
            else:
                # Añadir movimientos adicionales para la próxima iteración
                motion = add_random_walk_motion(get_motion_model())
        
        minp = float("inf")
        minix, miniy = -1, -1
        
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        
        if minix == -1 or miniy == -1:
            print("No valid move found!")
            break
            
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(goal_x - xp, goal_y - yp)
        path_x.append(xp)
        path_y.append(yp)

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.05)  # Reducir pausa para más fluidez
        
        # Reset motion to normal after successful move
        if len(motion) > 8:  # Si tenemos movimientos adicionales
            motion = get_motion_model()

    if d < reso:
        print("Goal reached!!")
    else:
        print(f"Stopped at distance {d:.2f} from goal after {iteration} iterations")

    return path_x, path_y


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def draw_robot_rectangle(x, y, width, length, minx, miny, reso, color='blue'):
    """
    Dibuja el rectángulo del robot en la visualización.
    """
    # Convertir coordenadas del mundo a coordenadas de la grilla
    ix = (x - minx) / reso
    iy = (y - miny) / reso
    iwidth = width / reso
    ilength = length / reso
    
    # Crear rectángulo centrado en (ix, iy)
    rect = plt.Rectangle((ix - iwidth/2, iy - ilength/2), iwidth, ilength, 
                        linewidth=2, edgecolor=color, facecolor='none', alpha=0.7)
    plt.gca().add_patch(rect)


def main():
    print("potential_field_planning start")

    start_x = -12.0  # start x position [m]
    start_y = -12.0  # start y position [m]
    goal_x = 9.0  # goal x position [m]
    goal_y = 7.0  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    
    # Dimensiones del robot rectangular (tractor + trailer)
    robot_width = ROBOT_WIDTH  # ancho del tractor + trailer [m]
    robot_length = ROBOT_LENGTH  # largo total del tractor + trailer [m]

    # Obstáculos van de izquierda a derecha, de abajo hacia arriba
    # obstacle x position list [m]
    obstacle_x = [-10.6006, ##/shape[3]
                      -8.3, ##/sofa[4]
                      -8.4, ##/sofa[3]
                     -8.25, ##/sofa[5]
                   -8.3506, ##/shape[1]
                    -7.450, ##/sofa[2]
                    -3.450, ##/sofa[0]

                    -1.125-2.5, ##/ConcretBlock[2]
                    -1.125,     ##/ConcretBlock[2]
                    -1.125+2.5, ##/ConcretBlock[2]

                    -1.075-2.5, ##/ConcretBlock[1]
                    -1.075,     ##/ConcretBlock[1]
                    -1.075+2.5, ##/ConcretBlock[1]

                    -1.000-2.5, ##/ConcretBlock[0]
                    -1.000,     ##/ConcretBlock[0]
                    -1.000+2.5, ##/ConcretBlock[0]

                     0.500, ##/sofa[1]
                     
                     6.700, ##/ConcretBlock[3]
                     6.700, ##/ConcretBlock[3]
                     6.700, ##/ConcretBlock[3]

                     6.725, ##/ConcretBlock[4]
                     6.725, ##/ConcretBlock[4]
                     6.725, ##/ConcretBlock[4]

                    5.9494, ##/shape[4]
                   10.1994, ##/shape[5]
                   10.2744  ##/shape[2]
                   ]   
    
    # obstacle y position list [m]
    obstacle_y = [-9.99567, ##/shape[3]
                  -9.60222, ##/sofa[4]
                   -6.1022, ##/sofa[3]
                   -2.3522, ##/sofa[5]
                   0.46433, ##/shape[1]
                   6.12278, ##/sofa[2]
                  6.172278, ##/sofa[0]

                    -9.725, ##/ConcretBlock[2]
                    -9.725, ##/ConcretBlock[2]
                    -9.725, ##/ConcretBlock[2]

                    -4.550, ##/ConcretBlock[1]
                    -4.550, ##/ConcretBlock[1]
                    -4.550, ##/ConcretBlock[1]

                     0.650, ##/ConcretBlock[0]
                     0.650, ##/ConcretBlock[0]
                     0.650, ##/ConcretBlock[0]

                   6.12278, ##/sofa[1]

                    -8.300-2.5, ##/ConcretBlock[3]
                    -8.300,     ##/ConcretBlock[3]
                    -8.300+2.5, ##/ConcretBlock[3]

                     0.675-2.5, ##/ConcretBlock[4]
                     0.675,     ##/ConcretBlock[4]
                     0.675+2.5, ##/ConcretBlock[4]

                   7.38933, ##/shape[4]
                   4.56443, ##/shape[5]
                  10.11433  ##/shape[2]
                  ]   
# Listado de obstáculos en orden:
##/shape[3]
##/sofa[4]
##/sofa[3]
##/sofa[5]
##/shape[1]
##/sofa[2]
##/sofa[0]
##/ConcretBlock[2]
##/ConcretBlock[1]
##/ConcretBlock[0]
##/sofa[1]
##/ConcretBlock[3]
##/ConcretBlock[4]
##/shape[4]
##/shape[5]
##/shape[2]


    if show_animation:
        plt.grid(False)
        plt.axis("equal")

    # path generation
    path_x, path_y = potential_field_planning(
        start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, grid_size, robot_width, robot_length)
    # print("path_x: ", path_x)
    # print("path_y: ", path_y)
    print("Camino generado:")
    for i in range(len(path_x)):
        print("({},{})".format(path_x[i], path_y[i]))
    # plt.plot(path_x, path_y, "-r", label="path")
    # plt.show
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")