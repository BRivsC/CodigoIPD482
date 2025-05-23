import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib import animation

# Parámetros del robot (Tabla 2 del paper)
b0 = b = 0.24       # m
d0 = d = 0.255      # m
r0 = r = 0.095      # m
L  = 0.6            # m
mr0 = mr = 16.0     # kg
mw0 = mw = 0.5      # kg
I0 = Irz0 = 0.537   # kg·m2
Iwz0 = Iwz = 0.0011 # kg·m2
Iwy0 = Iwy = 0.0023 # kg·m2

# Parámetros LuGre (Tabla 4)
sigma0, sigma1, sigma2 = 20.0, 5.0, 20.0
mu_c, mu_s = 0.28, 0.34
vs = 12.5

### Valores iniciales ###
# Vector q = [X0, Y0, θ0, η01, η02, ρ01, ρ02, φ01, φ02,
#             X ,  Y ,  θ ,  η1,   η2,   ρ1,   ρ2,   φ1,   φ2]
q0 = np.zeros(18)

# Posición inicial del tractor
q0[0] = 0.7    # X0
q0[1] = -0.3   # Y0
q0[2] = 0.0    # θ0 (rad)

# Posición inicial del remolque 
q0[9] = q0[0]  - L  # X (igual a X0)
q0[10] = q0[1]      # Y (L metros detrás del tractor)
q0[11] = q0[2]      # θ (mismo ángulo)

# IMPORTANTE: En el sistema de coordenadas donde θ0=0 significa que el tractor mira hacia +X:
# η01, η02 = desplazamiento lateral de las ruedas del tractor
q0[3] = q0[1] - d0     # η01 = Y0 - d0
q0[4] = q0[1] - d0     # η02 = Y0 - d0

# ρ01, ρ02 = desplazamiento longitudinal de las ruedas del tractor
q0[5] = q0[0] + b0     # ρ01 = X0 + b0 (rueda izquierda)
q0[6] = q0[0] - b0     # ρ02 = X0 - b0 (rueda derecha)

# φ01, φ02 = ángulos de dirección de las ruedas del tractor
q0[7] = 0.0    # φ01 (rad)
q0[8] = 0.0    # φ02 (rad)

# η1, η2 = desplazamiento lateral de las ruedas del remolque
q0[12] = q0[10] - d    # η1 = Y - d
q0[13] = q0[10] - d    # η2 = Y - d

# ρ1, ρ2 = desplazamiento longitudinal de las ruedas del remolque
q0[14] = q0[9] + b     # ρ1 = X + b (rueda izquierda)
q0[15] = q0[9] - b     # ρ2 = X - b (rueda derecha)

# φ1, φ2 = ángulos de dirección de las ruedas del remolque
q0[16] = 0.0   # φ1 (rad)
q0[17] = 0.0   # φ2 (rad)

# Vector dq = [dX0, dY0, dθ0, dη01, dη02, dρ01, dρ02, dφ01, dφ02,
#               dX , dY , dθ , dη1,   dη2,   dρ1,   dρ2,   dφ1,   dφ2]
dq0 = np.zeros(18)
# Podemos activar velocidades iniciales si se desea
# dq0[5] = 0.5  # dρ01
# dq0[6] = 0.5  # dρ02



M = np.diag(np.array([
    mr0, mr0, I0 + 2*Iwz,
    mw, mw, mw, mw,
    Iwy, Iwy,
    mr, mr, I0 + 2*Iwz,
    mw, mw, mw, mw,
    Iwy, Iwy
]))

B = np.zeros((18, 2))
B[5,0] = 1   # ρ₀₁ (rueda izquierda tractor) - primer torque
B[6,1] = 1   # ρ₀₂ (rueda derecha tractor) - segundo torque

# También podríamos impulsar las ruedas del remolque si queremos tracción total
# B[14,0] = 0.5  # ρ₁ (rueda izquierda remolque) - añadir parte del primer torque
# B[15,1] = 0.5  # ρ₂ (rueda derecha remolque) - añadir parte del segundo torque

def constraint_matrix(q):
    theta0 = q[2]  # Ángulo del tractor
    theta = q[11]  # Ángulo del remolque
    
    A = np.zeros((10, 18))
    
    # Primera fila
    A[0, 0] = np.cos(theta0)
    A[0, 1] = np.sin(theta0)
    A[0, 2] = b0
    A[0, 6] = -1
    
    # Segunda fila
    A[1, 0] = np.cos(theta0)
    A[1, 1] = np.sin(theta0)
    A[1, 2] = -b0
    A[1, 7] = -1
    
    # Tercera fila
    A[2, 0] = -np.sin(theta0)
    A[2, 1] = np.cos(theta0)
    A[2, 2] = -d0
    A[2, 3] = -1
    
    # Cuarta fila
    A[3, 0] = -np.sin(theta0)
    A[3, 1] = np.cos(theta0)
    A[3, 2] = -d0
    A[3, 4] = -1
    
    # Quinta fila
    A[4, 0] = 1
    A[4, 9] = -1
    
    # Sexta fila
    A[5, 1] = 1
    A[5, 2] = -d0*np.cos(theta0)
    A[5, 10] = -1
    A[5, 11] = -L*np.cos(theta)
    
    # Séptima fila
    A[6, 10] = np.cos(theta)
    A[6, 11] = np.sin(theta)
    A[6, 12] = b
    A[6, 15] = -1
    
    # Octava fila
    A[7, 10] = np.cos(theta)
    A[7, 11] = np.sin(theta)
    A[7, 12] = -b
    A[7, 16] = -1
    
    # Novena fila
    A[8, 10] = -np.sin(theta)
    A[8, 11] = np.cos(theta)
    A[8, 12] = -d
    A[8, 13] = -1
    
    # Décima fila
    A[9, 10] = -np.sin(theta)
    A[9, 11] = np.cos(theta)
    A[9, 12] = -d
    A[9, 14] = -1
    
    return A


def compute_dA_qdot(q, dq):
    """
    Calcula (dot A)·dq, donde A(q) es tu matriz de restricciones 10×18.
    Devuelve un array de forma (10,1).
    """
    theta0 = q[2]
    theta  = q[11]
    dtheta0 = dq[2]
    dtheta  = dq[11]
    
    # Derivadas parciales de A w.r.t theta0 y theta
    dA_dtheta0 = np.zeros((10,18))
    dA_dtheta  = np.zeros((10,18))
    
    # Parcial respecto a theta0 (columnas 0 y 1 aparecen cos(theta0), sin(theta0))
    # fila 0
    dA_dtheta0[0,0] = -np.sin(theta0)
    dA_dtheta0[0,1] =  np.cos(theta0)
    # fila 1
    dA_dtheta0[1,0] = -np.sin(theta0)
    dA_dtheta0[1,1] =  np.cos(theta0)
    # fila 2
    dA_dtheta0[2,0] = -np.cos(theta0)
    dA_dtheta0[2,1] = -np.sin(theta0)
    # fila 3
    dA_dtheta0[3,0] = -np.cos(theta0)
    dA_dtheta0[3,1] = -np.sin(theta0)
    # fila 5 (–d0·cos(theta0) en A[5,2])
    dA_dtheta0[5,2]  =  d0 * np.sin(theta0)
    
    # Parcial respecto a theta (remolque) — columnas 10 y 11
    # fila 5: A[5,11] = -L·cos(theta)
    dA_dtheta[5,11]  =  L * np.sin(theta)
    # fila 6 y 7: A[6,10]=cos(theta), A[6,11]=sin(theta)
    dA_dtheta[6,10] = -np.sin(theta)
    dA_dtheta[6,11] =  np.cos(theta)
    dA_dtheta[7,10] = -np.sin(theta)
    dA_dtheta[7,11] =  np.cos(theta)
    # fila 8 y 9: A[8,10]=-sin(theta), A[8,11]=cos(theta)
    dA_dtheta[8,10] = -np.cos(theta)
    dA_dtheta[8,11] = -np.sin(theta)
    dA_dtheta[9,10] = -np.cos(theta)
    dA_dtheta[9,11] = -np.sin(theta)
    
    # Construir dot A = (∂A/∂θ0)*dθ0 + (∂A/∂θ)*dθ
    dotA = dA_dtheta0 * dtheta0 + dA_dtheta * dtheta  # (10×18)
    return dotA @ dq   # devuelve shape (10,), no reshape

def calculate_lambda(q, dq, M, B, Tau, friction_forces, coriolis_forces):
    A = constraint_matrix(q)          # (10×18)
    dA_qdot = compute_dA_qdot(q, dq)  # (10,)

    tau_vec    = (B @ Tau).flatten()  # (18,)
    ext_forces = tau_vec + friction_forces + coriolis_forces  # (18,)

    # Ahora todo en 1-D:
    AMt = A @ np.linalg.solve(M, A.T)             # (10×10)
    RHS = - A @ np.linalg.solve(M, ext_forces)    # (10,)
    RHS -= dA_qdot                                # (10,)

    lambda_vec = np.linalg.solve(AMt, RHS)        # (10,)
    return lambda_vec

def calculate_constrained_acceleration(q, dq, M, B, Tau, friction_forces, coriolis_forces):
    tau_vec = (B @ Tau).flatten()   # (18,)
    forces  = tau_vec + friction_forces + coriolis_forces  # (18,)

    ddq_un = np.linalg.solve(M, forces)  # (18,)
    lambda_vec = calculate_lambda(q, dq, M, B, Tau, friction_forces, coriolis_forces)  # (10,)
    corr   = np.linalg.solve(M, constraint_matrix(q).T @ lambda_vec)  # (18,)

    return ddq_un + corr  # (18,) + (18,) → OK

def calculate_coriolis_matrix(dq):
    C = np.zeros((18))

    # Índices correctos para las velocidades
    # Tractor
    dX0 = dq[0] 
    dY0 = dq[1]
    dtheta0 = dq[2]
    detha01 = dq[3]
    detha02 = dq[4]
    drho01 = dq[5]
    drho02 = dq[6]
    dphi01 = dq[7]
    dphi02 = dq[8]
    
    # Remolque
    dX = dq[9]
    dY = dq[10]
    dtheta = dq[11]     # Ángulo del remolque
    deta1 = dq[12]
    deta2 = dq[13]
    drho1 = dq[14]
    drho2 = dq[15]
    dphi1 = dq[16]
    dphi2 = dq[17]

    # Tractor: términos para ruedas
    C[4] = -mw * drho01*dtheta0
    C[5] = -mw * drho02*dtheta0
    C[6] =  mw * detha01*dtheta0
    C[7] =  mw * detha02*dtheta0

    # Remolque: términos para ruedas (índices corregidos)
    C[13] = -mw * drho1*dtheta
    C[14] = -mw * drho2*dtheta
    C[15] =  mw * deta1*dtheta
    C[16] =  mw * deta2*dtheta

    # El resto de términos quedan en cero
    return C

# Modelo de fricción LuGre
# No se está usando!!!1
def lugre_force(v_rel, z):
    g = mu_c + (mu_s - mu_c) * np.exp(-(v_rel / vs)**2)
    dz = v_rel - (np.abs(v_rel)/g)*z
    F = sigma0 * z + sigma1 * dz + sigma2 * v_rel
    return F, dz


def calculate_friction(dq):
    friction = np.zeros((18))
    
    return friction

def system_dynamics(t, state, tau1, tau2):
    q = state[:18]
    dq = state[18:]
    
    Tau = np.array([[tau1], [tau2]])
    
    friction = calculate_friction(dq)
    coriolis = calculate_coriolis_matrix(dq)
    ddq = calculate_constrained_acceleration(q, dq, M, B, Tau, friction, coriolis)
    
    return np.concatenate([dq, ddq.flatten()])

def simulate_system(q0, dq0, t_span, tau1_func, tau2_func, dt=0.01):


    constraint_error = constraint_matrix(q0) @ q0
    print("Error de restricción en simulate_system:", np.linalg.norm(constraint_error))
    
    # NOTA: Saltamos la verificación por ahora
    # if np.linalg.norm(constraint_error) > 0.1:
    #     assert False, f"El estado inicial no cumple las restricciones holónomas: error={np.linalg.norm(constraint_error)}"
    
    state = np.concatenate([q0, dq0])
    # Puntos de tiempo para la simulación
    t_eval = np.arange(t_span[0], t_span[1], dt)
    
    # Función de dinámicas que se pasa a solve_ivp
    def dynamics_wrapper(t, state):
        tau1 = tau1_func(t)
        tau2 = tau2_func(t)
        return system_dynamics(t, state, tau1, tau2)
    
    solution = solve_ivp(
        dynamics_wrapper,
        t_span,
        state,
        t_eval=t_eval,
        method='RK45',
        rtol=1e-6,
        atol=1e-6
    )
    
    return solution

def plot_results(solution):
    t = solution.t
    states = solution.y
    # Estados: q = [X0, Y0, θ0, η01, η02, ρ01, ρ02, φ01, φ02,
    #               X ,  Y ,  θ ,  η1,   η2,   ρ1,   ρ2,   φ1,   φ2]
    
    # Extraer posiciones y ángulos relevantes
    x0 = states[0, :]     # Posición x del tractor
    y0 = states[1, :]     # Posición y del tractor

    x = states[9, :]     # Posición x del remolque
    y = states[10, :]     # Posición y del remolque
    theta0 = states[2, :] # Ángulo del tractor
    theta = states[11, :] # Ángulo del remolque

    #en grados
    theta0 = np.degrees(theta0)
    theta = np.degrees(theta)
    
    # Crear figura para la trayectoria
    plt.figure(figsize=(10, 8))
    
    # Graficar trayectoria
    plt.subplot(2, 1, 1)
    plt.plot(x0, y0, 'b-', linewidth=2, label='Trayectoria Tractor')
    plt.plot(x, y, 'r-', linewidth=2, label='Trayectoria Remolque')
    plt.grid(True)
    plt.xlabel('Posición X (m)')
    plt.ylabel('Posición Y (m)')
    plt.title('Trayectoria del Robot Tractor-Remolque')
    plt.legend()
    
    # Graficar ángulos
    plt.subplot(2, 1, 2)
    plt.plot(t, theta0, 'r-', label='Ángulo Tractor')
    plt.plot(t, theta, 'g-', label='Ángulo Remolque')
    plt.grid(True)
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Ángulo (grados)')
    plt.title('Evolución de los Ángulos')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

def run_simulation_example():
    T_SIM_MAX_S = 60 # Tiempo máximo de simulación (s)

    # Funciones de torque constante igual para ambas ruedas
    def tau1_func(t):
        return 0.1  # Constante para todo tiempo
    
    def tau2_func(t):
        return 0.1  # Igual a tau1 para movimiento recto
    
    # # Alternativa: prueba con valores diferentes para generar giros
    # def tau1_func(t):
    #     if t < 3:
    #         return 0.1  # Movimiento recto inicial
    #     elif t < 6:
    #         return 0.15  # Mayor torque en rueda izquierda -> giro a la derecha
    #     else:
    #         return 0.1  # Vuelve a movimiento recto
    # 
    # def tau2_func(t):
    #     if t < 3:
    #         return 0.1  # Movimiento recto inicial
    #     elif t < 6:
    #         return 0.05  # Menor torque en rueda derecha -> giro a la derecha
    #     else:
    #         return 0.1  # Vuelve a movimiento recto
    
    t_span = (0, T_SIM_MAX_S)
    
    print('Iniciando simulación...')
    solution = simulate_system(q0, dq0, t_span, tau1_func, tau2_func)
    
    print('Simulación completada. Mostrando resultados...')
    plot_results(solution)
    
    return solution

# Si se ejecuta este script directamente, correr la simulación
if __name__ == '__main__':
    run_simulation_example()
