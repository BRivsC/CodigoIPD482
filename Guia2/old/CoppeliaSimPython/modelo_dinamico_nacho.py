import numpy as np
from numpy import sin, cos
from scipy.linalg import null_space
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Parametros del robot
b0 = b = 0.24   # Media distancia entre centro de tractor/trailer y rueda
d0 = d = 0.255  # Distancia entre puntos C y P
L = 0.60        # Distancia entre puntos C y J
L0 = L          
d = 0.5         # Distancia entre puntos C y J  
r0 = r = 0.095  # Radio de ruedas de tractor/trailer

mr0 = 16.0  # Masa de tractor
mr = 16.0   # Masa de trailer
mw = 0.5    # Masa de ruedas

Irz0 = Irz = 0.537  # Momento de inercia de tractor/trailer en eje Z
Iwz0 = Iwz = 0.0011 # Momento de inercia de ruedas en eje Z
Iwy0 = Iwy = 0.0023 # Momento de inercia de ruedas en eje Y

# Matriz de inercias
M_diag = np.array([
    mr0, mr0, Irz0 + 2*Iwz, mw, mw, mw, mw, Iwy, Iwy,  # Tractor
    mr, mr, Irz + 2*Iwz, mw, mw, mw, mw, Iwy, Iwy        # Trailer
])
M = np.diag(M_diag)

# Matriz de entradas
B = np.zeros((18, 2))
B[7, 0] = 1.0  # torque rueda izquierda tractor
B[8, 1] = 1.0  # torque rueda derecha tractor

# Matriz de restricciones
def A_mat(q):
    theta_0 = q[2]
    theta = q[11]
    c0, s0 = cos(theta_0), sin(theta_0)
    c, s = cos(theta), sin(theta)
    A = np.zeros((10, 18)) # A es 10x18

    A[0, [0,1,2,5]] = [c0, s0,  b0, -1] # cambiado r0 por 1 y arreglado el índice
    A[1, [0,1,2,6]] = [c0, s0, -b0, -1] # cambiado r0 por 1 y arreglado el índice
    A[2, [0,1,2,3]] = [-s0, c0, -d0, -1]
    A[3, [0,1,2,4]] = [-s0, c0, -d0, -1]
    A[4, [0,2,9,11]] =  [1,d0*s0 + L0*s0,-1, L*s] # agregados valores que antes eran cero
    A[5, [1,2,10,11]] = [1,-d0*s0 - L0*c0,-1, -L*c] # agregados valores que antes eran cero
    A[6, [9,10,11,14]] = [c, s,  b, -1] # cambiado r por 1
    A[7, [9,10,11,15]] = [c, s, -b, -1] # cambiado r por 1
    A[8, [9,10,11,12]] = [-s, c, -d, -1]
    A[9, [9,10,11,13]] = [-s, c, -d, -1]

    return A

# Dinámica reducida
def rhs(t, y):
    q = y[:18]
    v = y[18:]
    τ = np.array([1.0, 1.0])  # torques constantes
    N = null_space(A_mat(q))
    Mred = N.T @ M @ N
    vdot_red = np.linalg.solve(Mred, N.T @ B @ τ)
    vdot = N @ vdot_red
    return np.concatenate((v, vdot))

# Condiciones iniciales
q0 = np.zeros(18)
v0 = np.zeros(18)
v0[7] = v0[8] = 0.1 / r  # velocidades angulares de ruedas tractor
y0 = np.concatenate((q0, v0))

# Integración
t_span = (0, 6)
t_eval = np.linspace(*t_span, 600)
sol = solve_ivp(rhs, t_span, y0, t_eval=t_eval, rtol=1e-8)

X_tr, Y_tr = sol.y[9], sol.y[10]
X_trac, Y_trac = sol.y[0], sol.y[1]

# Graficar trayectoria
plt.figure(figsize=(6,6))
plt.plot(X_trac, Y_trac, label="Tractor CG")
plt.plot(X_tr, Y_tr, label="Trailer CG")
plt.axis("equal")
plt.grid(True)
plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.title("Trayectoria del robot TTWMR")
plt.legend()
plt.tight_layout()
plt.show()