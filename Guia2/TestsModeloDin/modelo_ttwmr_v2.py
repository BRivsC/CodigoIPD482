import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Parámetros del robot tractor
m_t = 50     # Masa del tractor (kg)
I_t = 10     # Inercia del tractor (kg·m^2)
r = 0.1      # Radio de las ruedas (m)
b = 0.3      # Mitad de la distancia entre ruedas (m)

# Parámetros del remolque
m_r = 20     # Masa del remolque (kg)
I_r = 2      # Inercia del remolque (kg·m^2)
L = 1.0      # Longitud entre tractor y remolque (m)

# Parámetros LuGre (mismos para tractor y remolque)
sigma_0 = 100
sigma_1 = 1
sigma_2 = 0.5
Fs = 30
Fc = 18
vs = 0.01

# Tiempo de simulación
dt = 0.01
T_total = 10
t = np.arange(0, T_total + dt, dt)

# Estado inicial extendido: [x, y, θ, x_r, y_r, θ_r]
q = np.array([0.0, 0.0, 0.0, -L, 0.0, 0.0])
u = np.array([0.0, 0.0, 0.0])    # [v, ω_trac, ω_trailer]
# Estados internos LuGre: tractor y remolque
z_r, z_l = 0.0, 0.0
z_tr_r, z_tr_l = 0.0, 0.0

# Torques en el tractor
Tau_r = np.zeros(len(t))
Tau_l = np.zeros(len(t))
Tau_r[t < 1] = 1
Tau_l[t < 1] = 1
Tau_r[(t >= 3) & (t < 8)] = 0
Tau_l[(t >= 3) & (t < 8)] = 0

# Almacenamiento
Q = np.zeros((6, len(t)))
U = np.zeros((3, len(t)))

def N_matrix(q):
    theta = q[2]
    theta_r = q[5]
    ct, st = np.cos(theta), np.sin(theta)
    ctr, str_ = np.cos(theta_r), np.sin(theta_r)

    N = np.zeros((6, 3))
    # Tractor
    N[0, 0] = ct
    N[1, 0] = st
    N[2, 1] = 1
    # Remolque kinemática pasiva
    N[3, 0] = ct
    N[3, 1] = -L * st
    N[4, 0] = st
    N[4, 1] =  L * ct
    N[5, 2] = 1
    return N

def lugre_force(v_rel, z):
    g = Fc + (Fs - Fc) * np.exp(-(v_rel / vs)**2)
    dz = v_rel - (sigma_0 / g) * z
    F = sigma_0 * z + sigma_1 * dz + sigma_2 * v_rel
    return F, dz

# Matriz de masa extendida
M = np.diag([m_t, m_t, I_t, m_r, m_r, I_r])

def simulate():
    global q, u, z_r, z_l, z_tr_r, z_tr_l
    for k in range(len(t)):
        theta = q[2]
        theta_r = q[5]
        ct, st = np.cos(theta), np.sin(theta)
        ctr, str_ = np.cos(theta_r), np.sin(theta_r)

        # Cinemática
        N = N_matrix(q)
        q_dot = N @ u

        # Proyección tractor
        Uq = ct * q_dot[0] + st * q_dot[1]
        Vq = -st * q_dot[0] + ct * q_dot[1]
        omega = u[1]

        # Fricción LuGre tractor
        v_r = Uq + b * omega
        v_l = Uq - b * omega
        F_r, dz_r = lugre_force(v_r, z_r)
        F_l, dz_l = lugre_force(v_l, z_l)
        z_r += dz_r * dt
        z_l += dz_l * dt

        # Proyección remolque
        Uq_tr = ctr * q_dot[3] + str_ * q_dot[4]
        Vq_tr = -str_ * q_dot[3] + ctr * q_dot[4]
        omega_tr = u[2]

        # Fricción LuGre remolque
        v_tr_r = Uq_tr + b * omega_tr
        v_tr_l = Uq_tr - b * omega_tr
        F_tr_r, dz_tr_r = lugre_force(v_tr_r, z_tr_r)
        F_tr_l, dz_tr_l = lugre_force(v_tr_l, z_tr_l)
        z_tr_r += dz_tr_r * dt
        z_tr_l += dz_tr_l * dt

        # Fuerzas y momentos de fricción total
        F_x = F_r + F_l
        M_z = b * (F_r - F_l)
        F_tr_x = F_tr_r + F_tr_l
        M_z_tr = b * (F_tr_r - F_tr_l)

        f_fric = np.array([
            -F_x * ct,     # tractor X
            -F_x * st,     # tractor Y
            -M_z,          # tractor momento
            -F_tr_x * ctr, # trailer X
            -F_tr_x * str_,# trailer Y
            -M_z_tr       # trailer momento
        ])

        # Término de Coriolis
        Cq_dot = np.array([
            m_t * Vq * omega,
           -m_t * Uq * omega,
            0,
            m_r * Vq_tr * omega_tr,
           -m_r * Uq_tr * omega_tr,
            0
        ])

        # Matrices de entrada
        Bq = (1 / r) * np.array([
            [ct, ct],
            [st, st],
            [ b, -b],
            [0,  0],
            [0,  0],
            [0,  0]
        ])
        tau = np.array([Tau_r[k], Tau_l[k]])

        # Ecuaciones proyectadas
        M_u = N.T @ M @ N
        C_u = N.T @ Cq_dot
        f_u = N.T @ f_fric
        B_u = N.T @ Bq

        dot_u = np.linalg.solve(M_u, B_u @ tau - C_u - f_u)

        # Integración
        u += dot_u * dt
        q += q_dot * dt

        Q[:, k] = q
        U[:, k] = u

simulate()

# Graficar trayectoria
plt.figure(figsize=(6,6))
plt.plot(Q[0], Q[1], 'b-', label="Tractor")
plt.plot(Q[3], Q[4], 'r--', label="Remolque")
plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.title("Trayectoria tractor + remolque con LuGre en ambas")
plt.legend(); plt.grid(); plt.axis("equal")
plt.show()


def animate_trayectoria(Q, interval=20):
    fig, ax = plt.subplots()
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Trayectoria tractor + remolque (animación)")
    ax.grid()
    ax.axis("equal")
    ax.plot(Q[0, :], Q[1, :], 'b-', alpha=0.2)  # Trayectoria completa tractor
    ax.plot(Q[3, :], Q[4, :], 'r--', alpha=0.2) # Trayectoria completa remolque

    tractor_line, = ax.plot([], [], 'bo-', label="Tractor")
    remolque_line, = ax.plot([], [], 'ro--', label="Remolque")
    eje_line, = ax.plot([], [], 'k-', linewidth=2, label="Eje")
    ax.legend()

    def init():
        tractor_line.set_data([], [])
        remolque_line.set_data([], [])
        eje_line.set_data([], [])
        return tractor_line, remolque_line, eje_line

    def update(frame):
        # Trayectorias
        tractor_line.set_data(Q[0, :frame], Q[1, :frame])
        remolque_line.set_data(Q[3, :frame], Q[4, :frame])
        # Eje entre el centro del tractor y el remolque en el frame actual
        eje_line.set_data([Q[0, frame-1], Q[3, frame-1]], [Q[1, frame-1], Q[4, frame-1]])
        return tractor_line, remolque_line, eje_line

    ani = animation.FuncAnimation(
        fig, update, frames=Q.shape[1], init_func=init,
        interval=interval, blit=True, repeat=False
    )
    plt.show()

animate_trayectoria(Q)