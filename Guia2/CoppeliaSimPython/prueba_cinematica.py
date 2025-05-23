import numpy as np
import matplotlib.pyplot as plt

# Parámetros físicos
m_t = 50      # Masa del tractor
Iz_t = 10     # Inercia del tractor
m_r = 40      # Masa del tráiler
Iz_r = 8      # Inercia del tráiler
r = 0.1       # Radio de las ruedas
b = 0.3       # Mitad del ancho entre ruedas del tractor
L = 0.6       # Longitud entre pivote y centro del tráiler

# Matriz de masa (simplificada)
M = np.diag([m_t + m_r, m_t + m_r, Iz_t + Iz_r])

# Parámetros del modelo LuGre
sigma_0, sigma_1, sigma_2 = 100, 1, 0.5
Fs, Fc, vs = 30, 18, 0.01

# Tiempo de simulación
dt = 0.01
T_total = 10
t = np.arange(0, T_total + dt, dt)

# Estados: [x_t, y_t, θ_t, θ_r]
q = np.array([0.0, 0.0, 0.0, 0.0])
u = np.array([0.0, 0.0])  # [vel_lineal, vel_angular]
z_r, z_l = 0.0, 0.0

# Perfiles de torque
Tau_r = np.zeros(len(t))
Tau_l = np.zeros(len(t))
Tau_r[t < 1] = 1
Tau_l[t < 1] = 2

# Almacenamiento
Q = np.zeros((4, len(t)))
U = np.zeros((2, len(t)))

for k in range(len(t)):
    x_t, y_t, theta_t, theta_r = q
    v, omega = u

    # Cinemática del tractor
    N = np.array([[np.cos(theta_t), 0],
                  [np.sin(theta_t), 0],
                  [0, 1]])
    q_dot = N @ u

    # Cinemática del tráiler (simplificada)
    theta_dot_r = (v * np.sin(theta_r - theta_t)) / L

    # Velocidades de las ruedas
    v_r = v + b * omega
    v_l = v - b * omega

    # Fricción LuGre
    g_r = Fc + (Fs - Fc) * np.exp(-(v_r / vs)**2)
    g_l = Fc + (Fs - Fc) * np.exp(-(v_l / vs)**2)

    dz_r = v_r - (sigma_0 / g_r) * z_r
    dz_l = v_l - (sigma_0 / g_l) * z_l
    z_r += dz_r * dt
    z_l += dz_l * dt

    F_r = sigma_0 * z_r + sigma_1 * dz_r + sigma_2 * v_r
    F_l = sigma_0 * z_l + sigma_1 * dz_l + sigma_2 * v_l

    F_x = F_r + F_l
    M_z = b * (F_r - F_l)

    # Dinámica
    f_fric = np.array([-F_x * np.cos(theta_t), -F_x * np.sin(theta_t), -M_z])
    Cq_dot = np.array([0, 0, 0])  # Simplificado
    Bq = (1 / r) * np.array([[np.cos(theta_t), np.cos(theta_t)],
                             [np.sin(theta_t), np.sin(theta_t)],
                             [b, -b]])
    tau = np.array([Tau_r[k], Tau_l[k]])
    M_u = N.T @ M @ N
    C_u = N.T @ Cq_dot
    f_u = N.T @ f_fric
    B_u = N.T @ Bq

    dot_u = np.linalg.solve(M_u, B_u @ tau - C_u - f_u)
    u += dot_u * dt
    q[:3] += q_dot * dt
    q[3] += theta_dot_r * dt  # Actualización del ángulo del tráiler

    Q[:, k] = q
    U[:, k] = u

# Gráficas
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(t, U[0, :], label="Velocidad U [m/s]")
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, U[1, :], label="Velocidad ω [rad/s]")
plt.grid(True)
plt.legend()
plt.xlabel("Tiempo [s]")
plt.tight_layout()
plt.show()

# Trayectoria
plt.figure()
plt.plot(Q[0, :], Q[1, :], 'r-', label='Tractor')
plt.plot(Q[0, :] - L * np.cos(Q[3, :]), Q[1, :] - L * np.sin(Q[3, :]), 'b--', label='Tráiler')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid(True)
plt.title('Trayectoria del TTWMR')
plt.axis('equal')
plt.show()
