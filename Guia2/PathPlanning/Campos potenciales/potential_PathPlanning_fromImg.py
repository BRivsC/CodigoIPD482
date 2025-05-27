from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True

def load_obstacles_from_image(image_path, width_m=25.0, height_m=25.0):
    img = Image.open(image_path).convert('L')  # grayscale
    img = np.array(img)
    # Obstáculos: píxeles negros (valor bajo)
    obstacle_indices = np.where(img < 128)
    img_h, img_w = img.shape
    # Convertir píxeles a coordenadas en metros
    obstacle_x = obstacle_indices[1] * (width_m / img_w)
    obstacle_y = (img_h - obstacle_indices[0]) * (height_m / img_h)
    return obstacle_x.tolist(), obstacle_y.tolist()

def calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, rr, start_x, start_y):
    minx = min(min(obstacle_x), start_x, goal_x) - AREA_WIDTH / 2.0
    miny = min(min(obstacle_y), start_y, goal_y) - AREA_WIDTH / 2.0
    maxx = max(max(obstacle_x), start_x, goal_x) + AREA_WIDTH / 2.0
    maxy = max(max(obstacle_y), start_y, goal_y) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    pmap = [[0.0 for _ in range(yw)] for _ in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx
        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, goal_x, goal_y)
            uo = calc_repulsive_potential(x, y, obstacle_x, obstacle_y, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny

def calc_attractive_potential(x, y, goal_x, goal_y):
    return 0.5 * KP * np.hypot(x - goal_x, y - goal_y)

def calc_repulsive_potential(x, y, obstacle_x, obstacle_y, rr):
    if len(obstacle_x) == 0:
        return 0.0
    dists = np.hypot(x - np.array(obstacle_x), y - np.array(obstacle_y))
    dmin = np.min(dists)
    if dmin <= rr:
        dq = max(dmin, 0.1)
        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0

def get_motion_model():
    motion = [[1, 0], [0, 1], [-1, 0], [0, -1],
              [-1, -1], [-1, 1], [1, -1], [1, 1]]
    return motion

def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))
    if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
        previous_ids.popleft()
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

def potential_field_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, reso, rr):
    pmap, minx, miny = calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, rr, start_x, start_y)
    d = np.hypot(start_x - goal_x, start_y - goal_y)
    ix = round((start_x - minx) / reso)
    iy = round((start_y - miny) / reso)
    gix = round((goal_x - minx) / reso)
    giy = round((goal_y - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "oy")
        plt.plot(gix, giy, "ok")
        plt.annotate("GOAL", xy=(gix + 2, giy + 2))
        plt.annotate("START", xy=(ix + 2, iy + 2), color='yellow')
        plt.axis(False)

    path_x, path_y = [start_x], [start_y]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(goal_x - xp, goal_y - yp)
        path_x.append(xp)
        path_y.append(yp)

        if oscillations_detection(previous_ids, ix, iy):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.12)

    print("Goal!!")
    return path_x, path_y

def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

def main():
    print("potential_field_planning with image obstacles start")

    # Parámetros del escenario
    start_x = 2.0  # [m]
    start_y = 2.0  # [m]
    goal_x = 23.0  # [m]
    goal_y = 23.0  # [m]
    grid_size = 0.5  # [m]
    robot_radius = 1.0  # [m]

    # Cargar obstáculos desde imagen
    image_path = "CamPot_in1.png"  # Cambia por el nombre de tu imagen
    obstacle_x, obstacle_y = load_obstacles_from_image(image_path, width_m=25.0, height_m=25.0)

    if show_animation:
        plt.grid(False)
        plt.axis("equal")

    path_x, path_y = potential_field_planning(
        start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, grid_size, robot_radius)
    plt.plot(path_x, path_y, "-r", label="path")
    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()