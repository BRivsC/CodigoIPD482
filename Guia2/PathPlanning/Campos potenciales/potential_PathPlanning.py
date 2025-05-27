from collections import deque
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True


def calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, rr, start_x, start_y):
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
            uo = calc_repulsive_potential(x, y, obstacle_x, obstacle_y, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, goal_x, goal_y):
    return 0.5 * KP * np.hypot(x - goal_x, y - goal_y)


def calc_repulsive_potential(x, y, obstacle_x, obstacle_y, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(obstacle_x):
        d = np.hypot(x - obstacle_x[i], y - obstacle_y[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - obstacle_x[minid], y - obstacle_y[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


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

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def potential_field_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, reso, rr):
    # calc potential field
    pmap, minx, miny = calc_potential_field(goal_x, goal_y, obstacle_x, obstacle_y, reso, rr, start_x, start_y)

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
                p = float("inf")  # outside area
                print("outside potential!")
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

        if (oscillations_detection(previous_ids, ix, iy)):
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
    print("potential_field_planning start")

    start_x = -12.0  # start x position [m]
    start_y = -12.0  # start y position [m]
    goal_x = 9.0  # goal x position [m]
    goal_y = 7.0  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    # Obstáculos van de izquierda a derecha, de abajo hacia arriba
    obstacle_x = [-10.6006,     -8.3,    -8.4,   -8.25,  -8.3506,   -7.450,   -3.450, -1.125, -1.075, -1.000,   0.500,  6.700, 6.725,  5.9494, 10.1994, 10.2744 ]   # obstacle x position list [m]
    obstacle_y = [-9.99567, -9.60222, -6.1022, -2.3522,  0.46433,  6.12278, 6.172278, -9.725, -4.550,  0.650, 6.12278, -8.300, 0.675, 7.38933, 4.56443, 10.11433]   # obstacle y position list [m]

    if show_animation:
        plt.grid(False)
        plt.axis("equal")

    # path generation
    path_x, path_y = potential_field_planning(
        start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, grid_size, robot_radius)
    # plt.plot(path_x, path_y, "-r", label="path")
    # plt.show
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
7