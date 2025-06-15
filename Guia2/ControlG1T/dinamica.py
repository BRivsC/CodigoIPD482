import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
from matplotlib import animation

# ----------  PARÁMETROS (tabla 1 del paper) --------------------
P = dict(
    m1=4.0, m2=4.0, mw=0.2,           # masas [kg]
    I1=0.4, I2=0.4, Iw=0.2, Iwd=1e-4, # inercias [kg·m²]
    b=0.2,  r=0.1,                    # medio‐eje y radio rueda [m]
    L1=1.0, L2=1.0,                   # longitudes de barras [m]
    mu=-0.555                          # steering pasivo μ
)

# ---------- parámetros LuGre (longitudinal) --------------------
g            = 9.81
q0, q1, q2   = 2, 0.5, 2        # [N/m], [N·s/m], visc.[N·s/m]
μk, μs, vs   = 1, 0.5, 1       # fricción cinética / estática
Nwheel       = 0.5*(P['m1']+2*P['mw'])*g  # carga normal por rueda [N]
Fc, Fs       = μk*Nwheel, μs*Nwheel   # fuerzas límite [N]

def g_lugre(v):
    """Curva de Stribeck para la transición estático-dinámico."""
    return Fc + (Fs - Fc)*ca.exp(-(v/vs)**2)

# ---------- cinemática auxiliar -------------------------------
def beta(th1, th2):
    return P['mu'] * (th1 - th2)

# ---------- inertia matrix 6x6 ------------
def M_full(q):
    m1,m2,mw,I1,I2,Iw,Iwd,b,r,L1,L2,mu = \
        P['m1'],P['m2'],P['mw'],P['I1'],P['I2'],P['Iw'],P['Iwd'],P['b'],P['r'],P['L1'],P['L2'],P['mu']
    th1,th2 = q[2],q[3]
    be = beta(th1,th2)

    a1 = m1+m2+4*mw + 2*Iwd/r**2
    a2 = -(m2+2*mw+2*Iwd/r**2)*L2*mu*ca.sin(th2-be)
    a3 =  (m2+2*mw+2*Iwd/r**2)*L2*(1+mu)*ca.sin(th2-be)
    a4 = a1
    a5 =  (m2+2*mw+2*Iwd/r**2)*L2*mu*ca.cos(th2-be)
    a6 = -(m2+2*mw+2*Iwd/r**2)*L2*(1+mu)*ca.cos(th2-be)
    a7 = (I1+2*Iw+2*mw*b**2 + (m2+2*mw+2*Iwd/r**2)*
          (L1**2+L2**2*mu**2-2*L1*L2*mu*ca.cos(th1-th2+be)))
    a8 = (m2+2*mw+2*Iwd/r**2)*(L1*L2*(1+mu)*ca.cos(th1-th2+be)-mu*(1+mu)*L2**2)
    a9 = (I2+2*Iw+2*mw*b**2+2*Iwd*b**2/r**2+
          (m2+2*mw+2*Iwd/r**2)*L2**2*(1+mu)**2)
    a10=a11=Iwd
    return ca.vertcat(
        ca.horzcat(a1, 0, a2, a3, 0, 0),
        ca.horzcat(0, a4, a5, a6, 0, 0),
        ca.horzcat(a2, a5, a7, a8, 0, 0),
        ca.horzcat(a3, a6, a8, a9, 0, 0),
        ca.horzcat(0, 0, 0, 0, a10, 0),
        ca.horzcat(0, 0, 0, 0, 0, a11)
    )

# ---------- S(q) --------------------------
def S(q):
    th1,th2=q[2],q[3]
    b,r,L1,L2,mu=P['b'],P['r'],P['L1'],P['L2'],P['mu']
    be=beta(th1,th2); cbe=ca.cos(be)
    s12=ca.sin(th1-th2); c12=ca.cos(th1-th2)
    row4_0 = s12/(L2*(1+mu)*cbe)
    row4_1 = (L2*mu*cbe-L1*c12)/(L2*(1+mu)*cbe)
    return ca.vertcat(
        ca.horzcat(ca.cos(th1), 0),
        ca.horzcat(ca.sin(th1), 0),
        ca.horzcat(0, 1),
        ca.horzcat(row4_0, row4_1),
        ca.horzcat(1/r, b/r),
        ca.horzcat(1/r, -b/r)
    )

# speed of trailer yaw w2 from theorem 1
def w2_from_V(q,V):
    v1,w1=V[0],V[1]
    th1,th2=q[2],q[3]
    L1,L2,mu = P['L1'],P['L2'],P['mu']
    be=beta(th1,th2)
    cbe=ca.cos(be)
    num = v1*ca.sin(th1-th2)+w1*(L2*mu*cbe-L1*ca.cos(th1-th2))
    den = L2*(1+mu)*cbe
    return num/den

# ---------- Sdot(q,V) ---------------------
def Sdot(q,V):
    v1,w1 = V[0],V[1]
    w2 = w2_from_V(q,V)
    th1,th2 = q[2],q[3]
    b,r,L1,L2,mu=P['b'],P['r'],P['L1'],P['L2'],P['mu']
    be=beta(th1,th2); cbe=ca.cos(be); sbe=ca.sin(be)
    s12=ca.sin(th1-th2); c12=ca.cos(th1-th2)
    
    # derivative of row4 wrt theta1 and theta2
    drow4_dth1_0 = (c12*(1+mu)*cbe - s12*(-mu*sbe))/(L2*(1+mu)*cbe)**2
    drow4_dth1_1 = (-L2*mu*(-mu*sbe)-( -s12)*L1*(1) )/(L2*(1+mu)*cbe) - \
                   (L2*mu*cbe - L1*c12)*(-sbe*mu)/(L2*(1+mu)*cbe)**2
    
    drow4_dth2_0 = -drow4_dth1_0  # derivative wrt theta2 (opposite sign approx)
    drow4_dth2_1 = -drow4_dth1_1
    
    # Sdot = dS/dtheta1 * w1 + dS/dtheta2 * w2
    Sdot_result = ca.vertcat(
        ca.horzcat(-ca.sin(th1)*w1, 0),
        ca.horzcat(ca.cos(th1)*w1, 0),
        ca.horzcat(0, 0),
        ca.horzcat(drow4_dth1_0*w1 + drow4_dth2_0*w2, drow4_dth1_1*w1 + drow4_dth2_1*w2),
        ca.horzcat(0, 0),
        ca.horzcat(0, 0)
    )
    return Sdot_result

# ---------- Coriolis full 6x6 -------------
def C_full(q,V):
    v1,w1 = V[0],V[1]
    th1,th2 = q[2],q[3]
    m2,mw,Iwd,r,L1,L2,mu = P['m2'],P['mw'],P['Iwd'],P['r'],P['L1'],P['L2'],P['mu']
    be = beta(th1,th2)
    base = (m2+2*mw+2*Iwd/r**2)
    b1 = base*L2*(mu**2*v1 - 2*mu*(1+mu)*w1)*ca.cos(th2-be)
    b2 = base*L2*(1+mu)**2*w1*ca.cos(th2-be)
    b3 = base*L2*(mu**2*v1 - 2*mu*(1+mu)*w1)*ca.sin(th2-be)
    b4 = base*L2*(1+mu)**2*w1*ca.sin(th2-be)
    b5 = base*L1*L2*(mu+mu**2)*(v1-2*w1)*ca.sin(th1-th2+be)
    b6 = base*L1*L2*(1+mu)**2*w1*ca.sin(th1-th2+be)
    b7 = -base*L1*L2*(1+mu)*w1*ca.sin(th1-th2+be)
    
    C = ca.SX.zeros(6,6)
    C[0,2]=b1; C[0,3]=b2
    C[1,2]=b3; C[1,3]=b4
    C[2,2]=b5; C[2,3]=b6
    C[3,2]=b7
    return C

# ---------- TORQUE DE ACCIONAMIENTO ---------------------------
def T_drive(t):
    """Par en ruedas (N·m) → vector de fuerzas generalizadas en espacio V."""
    # Torques más realistas para movimiento suave
    tau_r = ca.if_else(t < 3.0, 0.6, 0.0)  # Reducido de 5.0 a 0.6
    tau_l = ca.if_else(t < 3.0, 0.4, 0.0)  # Reducido de 1.0 a 0.4
    b, r = P['b'], P['r']
    return ca.vertcat((tau_r + tau_l)/r, b*(tau_r - tau_l)/r)

# ---------- SISTEMA DINÁMICO SIMBÓLICO CON CASADI --------------
def create_dynamics():
    """Crea la función dinámica simbólica usando CasADi."""
    # Variables simbólicas del estado
    X = ca.SX.sym('X', 9)  # [q(6), V(2), z(1)]
    t = ca.SX.sym('t')     # tiempo
    
    q = X[:6]
    V = X[6:8]
    z = X[8]
    v1, w1 = V[0], V[1]
    
    # --- dinámica interna LuGre -------------------------------
    g_v = g_lugre(v1)                # [N]
    zdot = v1 - (ca.fabs(v1)/g_v)*z
    F_lu = q0*z + q1*zdot + q2*v1     # fuerza longitudinal [N]
    
    # --- dinámica proyectada del vehículo ---------------------
    S_q = S(q)
    Sdot_q = Sdot(q, V)
    M = M_full(q)
    C = C_full(q, V)
    
    # fuerzas/pares en coordenadas V -> restamos fricción
    Tau = T_drive(t)
    Tau_friction = ca.vertcat(Tau[0] - F_lu * ca.sign(v1), Tau[1])
    
    M1 = ca.mtimes(S_q.T, ca.mtimes(M, S_q))
    C1 = ca.mtimes(S_q.T, ca.mtimes(M, Sdot_q) + ca.mtimes(C, S_q))
    dV = ca.solve(M1, Tau_friction - ca.mtimes(C1, V))
    dq = ca.mtimes(S_q, V)
    
    # Derivada completa del estado
    dX = ca.vertcat(dq, dV, zdot)
    
    # Crear función CasADi
    dynamics = ca.Function('dynamics', [t, X], [dX])
    return dynamics

if __name__ == "__main__":
    # -------------- SIMULACIÓN CON CASADI ------------------------------------
    print("Creando sistema dinámico...")
    dynamics = create_dynamics()
    
    # Configuración de la simulación
    t0, tf = 0.0, 6.0
    N = 600  # número de puntos
    dt = (tf - t0) / (N - 1)
    
    # Crear variables simbólicas para el integrador
    x = ca.SX.sym('x', 9)
    t = ca.SX.sym('t')
    
    # Definir el sistema DAE para el integrador
    dae = {
        'x': x,
        't': t,
        'ode': dynamics(t, x)
    }
    
    integrator = ca.integrator('integrator', 'rk', dae, {'tf': dt})
    
    # Estado inicial
    X0 = np.zeros(9)
    
    # Simulación paso a paso
    print("Ejecutando simulación...")
    t_eval = np.linspace(t0, tf, N)
    sol_y = np.zeros((9, N))
    sol_y[:, 0] = X0
    
    X_current = X0
    t_current = t0
    
    for i in range(1, N):
        # Usar la API correcta: solo pasamos x0 como estado inicial
        # El integrador usa el dt configurado en opts
        result = integrator(x0=X_current)
        X_current = np.array(result['xf']).flatten()
        t_current = t_current + dt
        sol_y[:, i] = X_current
        
        # Progreso cada 100 pasos
        if i % 100 == 0:
            print(f"Paso {i}/{N}, t = {t_current:.2f}s")
    
    # Extraer variables para compatibilidad con el código de plotting
    x1, y1, th1, th2 = sol_y[0], sol_y[1], sol_y[2], sol_y[3]
    v1 = sol_y[6]
    
    print("Simulación completada!")

    # ---------- GRÁFICA DE VELOCIDAD---------------
    plt.figure(figsize=(8, 4))
    plt.plot(t_eval, v1, label='v₁(t)')
    plt.title('Velocidad longitudinal')
    plt.xlabel('t [s]')
    plt.grid()
    plt.legend()

    # --- posición del tractor y del tráiler ---
    xH = x1 - P['L1']*np.cos(th1)         # enganche
    yH = y1 - P['L1']*np.sin(th1)

    xB = xH - P['L2']*np.cos(th2)         # centro tráiler
    yB = yH - P['L2']*np.sin(th2)

    # --- gráfica ---
    plt.figure(figsize=(6,6))
    plt.plot(x1, y1, label='Tractor')     # trayectoria tractor
    plt.plot(xB, yB, label='Trailer')     # trayectoria tráiler
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Trayectorias en planta')
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()

    # ---------- ANIMACIÓN  ----------------------------------------
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    ax.grid()
    ax.plot(x1, y1, 'r--', lw=0.8, label='Trayectoria')
    tractor_line, = ax.plot([], [], 'b-', lw=3, label='Tractor')
    trailer_line, = ax.plot([], [], 'g-', lw=3, label='Trailer')
    ax.legend(loc='upper left')
    ax.set_xlim(x1.min()-0.5, x1.max()+0.5)
    ax.set_ylim(y1.min()-0.5, y1.max()+0.5)
    L_draw = 0.4

    def init():
        tractor_line.set_data([], [])
        trailer_line.set_data([], [])
        return tractor_line, trailer_line

    def animate(i):
        xT, yT, thT = x1[i], y1[i], th1[i]
        be = beta(th1[i], th2[i])
        xB = xT - P['L1']*np.cos(thT) - P['L2']*np.cos(th2[i]-be)
        yB = yT - P['L1']*np.sin(thT) - P['L2']*np.sin(th2[i]-be)
        # Tractor
        tractor_line.set_data([xT - L_draw*np.cos(thT)/2, xT + L_draw*np.cos(thT)/2],
                              [yT - L_draw*np.sin(thT)/2, yT + L_draw*np.sin(thT)/2])
        # Trailer
        trailer_line.set_data([xB - L_draw*np.cos(th2[i])/2, xB + L_draw*np.cos(th2[i])/2],
                              [yB - L_draw*np.sin(th2[i])/2, yB + L_draw*np.sin(th2[i])/2])
        return tractor_line, trailer_line

    ani = animation.FuncAnimation(fig, animate, frames=len(t_eval),
                                  init_func=init, interval=40, blit=True, repeat=False)
    plt.show()