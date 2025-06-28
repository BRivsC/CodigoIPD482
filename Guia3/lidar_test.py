import time
import math
from components.simulation import SimulationEnvironment
from components.sensors import GPSSensor
from components.robot_controller import RobotController
from components.visualization import TrajectoryVisualizer
import matplotlib.pyplot as plt
import numpy as np

import zmq
import struct
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:23000")
socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all topics

print("Esperando datos del LIDAR...")

try:
    while True:
        msg = socket.recv()
        # Los datos vienen como cadena binaria con estructura [x1 y1 z1 x2 y2 z2 ...]
        # Se pueden decodificar como floats (4 bytes cada uno)
        num_floats = len(msg) // 4
        float_data = struct.unpack('f' * num_floats, msg)
        points = np.array(float_data).reshape(-1, 3)

        print("Nube de puntos recibida:", points.shape)
except KeyboardInterrupt:
    print("Terminado por el usuario.")
finally:
    socket.close()
    context.term()
