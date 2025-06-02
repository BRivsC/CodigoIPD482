"""
Modelo dinámico completo sistema G1T
"""
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle, Circle                  #  ### NEW
import matplotlib.transforms as transforms             #  ### NEW

# ----------  PARÁMETROS (tabla 1 del paper) --------------------
P = dict(
    m1=16.0, m2=16.0, mw=1.5,           # masas [kg]
    I1=0.065, I2=0.065, Iw=0.02057, Iwd=1e-5, # inercias [kg·m²]
    b=0.2,  r=0.1,                    # medio‐eje y radio rueda [m]
    L1=0.5, L2=0.5,                   # longitudes de barras [m]
    mu=-0.45                   # steering pasivo μ
)

# ---------- parámetros LuGre (longitudinal) --------------------
g            = 9.81
σ0, σ1, σ2   = 6, 3.5, 6        # [N/m], [N·s/m], visc.[N·s/m]
μk, μs, vs   = 0.04, 0.045, 4     # fricción cinética / estática
Nwheel       = 0.5*(P['m1']+2*P['mw'])*g  # carga normal por rueda [N]
Fc, Fs       = μk*Nwheel, μs*Nwheel   # fuerzas límite [N]

def g_lugre(v):
    """Curva de Stribeck para la transición estático-dinámico."""
    return Fc + (Fs - Fc)*np.exp(-(v/vs)**2)

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
    a2 = -(m2+2*mw+2*Iwd/r**2)*L2*mu*np.sin(th2-be)
    a3 =  (m2+2*mw+2*Iwd/r**2)*L2*(1+mu)*np.sin(th2-be)
    a4 = a1
    a5 =  (m2+2*mw+2*Iwd/r**2)*L2*mu*np.cos(th2-be)
    a6 = -(m2+2*mw+2*Iwd/r**2)*L2*(1+mu)*np.cos(th2-be)
    a7 = (I1+2*Iw+2*mw*b**2 + (m2+2*mw+2*Iwd/r**2)*
          (L1**2+L2**2*mu**2-2*L1*L2*mu*np.cos(th1-th2+be)))
    a8 = (m2+2*mw+2*Iwd/r**2)*(L1*L2*(1+mu)*np.cos(th1-th2+be)-mu*(1+mu)*L2**2)
    a9 = (I2+2*Iw+2*mw*b**2+2*Iwd*b**2/r**2+
          (m2+2*mw+2*Iwd/r**2)*L2**2*(1+mu)**2)
    a10=a11=Iwd
    return np.array([[a1,0,a2,a3,0,0],
                     [0,a4,a5,a6,0,0],
                     [a2,a5,a7,a8,0,0],
                     [a3,a6,a8,a9,0,0],
                     [0,0,0,0,a10,0],
                     [0,0,0,0,0,a11]])

# ---------- S(q) --------------------------
def S(q):
    th1,th2=q[2],q[3]
    b,r,L1,L2,mu=P['b'],P['r'],P['L1'],P['L2'],P['mu']
    be=beta(th1,th2); cbe=np.cos(be)
    s12=np.sin(th1-th2); c12=np.cos(th1-th2)
    row4=[s12/(L2*(1+mu)*cbe),
          (L2*mu*cbe-L1*c12)/(L2*(1+mu)*cbe)]
    return np.array([[np.cos(th1),0],
                     [np.sin(th1),0],
                     [0,1],
                     row4,
                     [1/r, b/r],
                     [1/r,-b/r]])

# speed of trailer yaw w2 from theorem 1
def w2_from_V(q,V):
    v1,w1=V
    th1,th2=q[2],q[3]
    L1,L2,mu,Pcos = P['L1'],P['L2'],P['mu'],np.cos
    be=beta(th1,th2)
    cbe=np.cos(be)
    num = v1*np.sin(th1-th2)+w1*(L2*mu*cbe-L1*np.cos(th1-th2))
    den = L2*(1+mu)*cbe
    return num/den

# ---------- Sdot(q,V) ---------------------
def Sdot(q,V):
    v1,w1 = V
    w2 = w2_from_V(q,V)
    th1,th2 = q[2],q[3]
    mu=L2=P['L2']
    # partial derivatives
    dth1 = w1
    dth2 = w2
    b,r,L1,L2,mu=P['b'],P['r'],P['L1'],P['L2'],P['mu']
    be=beta(th1,th2); cbe=np.cos(be); sbe=np.sin(be)
    s12=np.sin(th1-th2); c12=np.cos(th1-th2)
    # derivative of row4 wrt theta1 and theta2
    drow4_dth1 = np.array([
        (c12*(1+mu)*cbe - s12*(-mu*sbe))/(L2*(1+mu)*cbe)**2,
        (-L2*mu*(-mu*sbe)-( -s12)*L1*(1) )/(L2*(1+mu)*cbe) - \
        (L2*mu*cbe - L1*c12)*(-sbe*mu)/(L2*(1+mu)*cbe)**2
    ])
    drow4_dth2 = -drow4_dth1  # derivative wrt theta2 (opposite sign approx)
    # Sdot = dS/dtheta1 * w1 + dS/dtheta2 * w2
    Sdot = np.zeros((6,2))
    # rows 0 and 1
    Sdot[0,0] = -np.sin(th1)*w1
    Sdot[1,0] =  np.cos(th1)*w1
    # row3
    Sdot[2,1] = 0
    # row4
    Sdot[3,:] = drow4_dth1*w1 + drow4_dth2*w2
    # rows 5,6 zeros
    return Sdot

# ---------- Coriolis full 6x6 -------------
def C_full(q,V):
    v1,w1 = V
    th1,th2 = q[2],q[3]
    m2,mw,Iwd,r,L1,L2,mu = P['m2'],P['mw'],P['Iwd'],P['r'],P['L1'],P['L2'],P['mu']
    be = beta(th1,th2)
    base = (m2+2*mw+2*Iwd/r**2)
    b1 = base*L2*(mu**2*v1 - 2*mu*(1+mu)*w1)*np.cos(th2-be)
    b2 = base*L2*(1+mu)**2*w1*np.cos(th2-be)
    b3 = base*L2*(mu**2*v1 - 2*mu*(1+mu)*w1)*np.sin(th2-be)
    b4 = base*L2*(1+mu)**2*w1*np.sin(th2-be)
    b5 = base*L1*L2*(mu+mu**2)*(v1-2*w1)*np.sin(th1-th2+be)
    b6 = base*L1*L2*(1+mu)**2*w1*np.sin(th1-th2+be)
    b7 = -base*L1*L2*(1+mu)*w1*np.sin(th1-th2+be)
    C=np.zeros((6,6))
    C[0,2]=b1; C[0,3]=b2
    C[1,2]=b3; C[1,3]=b4
    C[2,2]=b5; C[2,3]=b6
    C[3,2]=b7
    return C
# ---------- TORQUE DE ACCIONAMIENTO ---------------------------
def T_drive(t):
    """Par en ruedas (N·m) → vector de fuerzas generalizadas en espacio V."""
    # 1 s con torque, luego cero para ver el frenado LuGre
    tau_r, tau_l = (6, 1) if t < 6 else (0.0, 0.0)
    b, r = P['b'], P['r']
    return np.array([(tau_r + tau_l)/r,
                     b*(tau_r - tau_l)/r])

# ---------- DINÁMICA CON LUGRE --------------------------------
def f(t, X):
    """
    Estado:
        q  = X[0:6]  (x, y, θ1, θ2, φr, φl)
        V  = X[6:8]  (v1, ω1)
        z  = X[8]    (desplazamiento LuGre)
    """
    q   = X[:6]
    V   = X[6:8]
    z   = X[8]

    v1, w1 = V
    # --- dinámica interna LuGre -------------------------------
    g_v  = g_lugre(v1)                # [N]
    zdot = v1 - (abs(v1)/g_v)*z
    F_lu = σ0*z + σ1*zdot + σ2*v1     # fuerza longitudinal [N]

    # --- dinámica proyectada del vehículo ---------------------
    S_q     = S(q)
    Sdot_q  = Sdot(q, V)
    M       = M_full(q)
    C       = C_full(q, V)

    #  fuerzas/pares en coordenadas V  ->  restamos fricción
    Tau     = T_drive(t)
    Tau[0] -= F_lu * np.sign(v1)                    # freno LuGre

    M1   = S_q.T @ M @ S_q
    C1   = S_q.T @ (M @ Sdot_q + C @ S_q)
    dV   = np.linalg.solve(M1, Tau - C1 @ V)
    dq   = S_q @ V

    return np.hstack((dq, dV, zdot))

# -------------- SIMULACIÓN ------------------------------------
t_eval = np.linspace(0, 6, 600)
X0     = np.zeros(9)   # estado inicial (z = 0)

sol = solve_ivp(f, (t_eval[0], t_eval[-1]), X0, t_eval=t_eval, method='RK45')

x1, y1, th1, th2 = sol.y[0], sol.y[1], sol.y[2], sol.y[3]


# ---------- GRÁFICA DE VELOCIDAD---------------
v1              = sol.y[6]

plt.figure(figsize=(8, 4))
plt.plot(sol.t, v1, label='v₁(t)')
plt.title('Velocidad longitudinal'); plt.xlabel('t [s]'); plt.grid(); plt.legend()


# --- posición del tractor y del tráiler ---
xH = x1 - P['L1']*np.cos(th1)         # enganche
yH = y1 - P['L1']*np.sin(th1)

xB = xH - P['L2']*np.cos(th2)         # centro tráiler
yB = yH - P['L2']*np.sin(th2)

# --- gráfica ---
plt.figure(figsize=(6,6))
plt.plot(x1, y1, label='Tractor')     # trayectoria tractor
plt.plot(xB, yB, label='Trailer')     # trayectoria tráiler
plt.xlabel('x [m]'); plt.ylabel('y [m]')
plt.title('Trayectorias en planta')
plt.axis('equal'); plt.grid(); plt.legend()
plt.show()






# ---------- ANIMACIÓN  ----------------------------------------
fig, ax = plt.subplots(figsize=(6, 6)); ax.set_aspect('equal'); ax.grid()
ax.plot(x1, y1, 'r--', lw=0.8, label='Trayectoria')
ax.plot(0,0, color='red', lw=1.8, label='Tractor')
ax.plot(0,0,color='blue', lw=1.8, label='Trailer')

# --- cuerpos ---------------------------------------------------
body_size = 0.40
tractor_body = Rectangle((-body_size/2, -body_size/2),
                         body_size, body_size,
                         lw=2, edgecolor='red', facecolor='none')
trailer_body = Rectangle((-body_size/2, -body_size/2),
                         body_size, body_size,
                         lw=2, edgecolor='blue', facecolor='none')
ax.add_patch(tractor_body)
ax.add_patch(trailer_body)

# --- barras ----------------------------------------------------
bar_w = 0.08
bar1 = Rectangle((-P['L1']/2, -bar_w/2), P['L1'], bar_w,
                 lw=0, facecolor='gray')
bar2 = Rectangle((-P['L2']/2, -bar_w/2), P['L2'], bar_w,
                 lw=0, facecolor='gray')
ax.add_patch(bar1);  ax.add_patch(bar2)

# --- ruedas → rectángulos (“bloques” negros) ------------------ ### EDIT
wheel_w, wheel_h = 0.2, 0.12          # ancho vs alto visibles
w1_L = Rectangle((-wheel_w/2, -wheel_h/2), wheel_w, wheel_h, fc='k')
w1_R = Rectangle((-wheel_w/2, -wheel_h/2), wheel_w, wheel_h, fc='k')
w2_L = Rectangle((-wheel_w/2, -wheel_h/2), wheel_w, wheel_h, fc='k')
w2_R = Rectangle((-wheel_w/2, -wheel_h/2), wheel_w, wheel_h, fc='k')
for w in (w1_L, w1_R, w2_L, w2_R): ax.add_patch(w)

# --- “pelotita” en el enganche ------------------------------- ### EDIT
pivot_ball = Circle((0, 0), 0.05, fc='red', ec='none')
ax.add_patch(pivot_ball)

ax.legend(loc='upper left')
ax.set_xlim(x1.min()-0.8, x1.max()+0.8)
ax.set_ylim(y1.min()-0.8, y1.max()+0.8)

def init():
    return (tractor_body, trailer_body, bar1, bar2,
            w1_L, w1_R, w2_L, w2_R, pivot_ball)        ### EDIT

def animate(i):
    # ----- posiciones clave -----
    xT, yT, thT = x1[i], y1[i], th1[i]
    xH = xT - P['L1']*np.cos(thT)
    yH = yT - P['L1']*np.sin(thT)
    be = P['mu']*(th1[i]-th2[i])
    xB = xH - P['L2']*np.cos(th2[i]-be)
    yB = yH - P['L2']*np.sin(th2[i]-be)

    # ----- transformaciones para cuerpos -----
    tr_T = transforms.Affine2D().rotate(thT).translate(xT, yT) + ax.transData
    tr_B = transforms.Affine2D().rotate(th2[i]).translate(xB, yB) + ax.transData
    tractor_body.set_transform(tr_T)
    trailer_body.set_transform(tr_B)

    # ----- barras -----
    alpha2 = np.arctan2(yB - yH, xB - xH) 

    # ----- barras -----
    tr_bar1 = (transforms.Affine2D()
               .rotate(thT)
               .translate((xT + xH)/2, (yT + yH)/2)
               + ax.transData)

    tr_bar2 = (transforms.Affine2D()
               .rotate(alpha2)              # ← usa alpha2 en vez de th2[i]
               .translate((xH + xB)/2, (yH + yB)/2)
               + ax.transData)

    bar1.set_transform(tr_bar1)
    bar2.set_transform(tr_bar2)

    # ----- ruedas (rectángulos) -----                               ### EDIT
    offset_T = np.array([-np.sin(thT),  np.cos(thT)]) * (body_size/2 + wheel_h/2)
    offset_B = np.array([-np.sin(th2[i]), np.cos(th2[i])]) * (body_size/2 + wheel_h/2)
    # tractor
    w1_L.set_transform(transforms.Affine2D().rotate(thT)
                       .translate(*(np.array([xT, yT]) + offset_T)) + ax.transData)
    w1_R.set_transform(transforms.Affine2D().rotate(thT)
                       .translate(*(np.array([xT, yT]) - offset_T)) + ax.transData)
    # trailer
    w2_L.set_transform(transforms.Affine2D().rotate(th2[i])
                       .translate(*(np.array([xB, yB]) + offset_B)) + ax.transData)
    w2_R.set_transform(transforms.Affine2D().rotate(th2[i])
                       .translate(*(np.array([xB, yB]) - offset_B)) + ax.transData)

    # ----- pivot ball -----
    pivot_ball.center = (xH, yH)

    return (tractor_body, trailer_body, bar1, bar2,
            w1_L, w1_R, w2_L, w2_R, pivot_ball)

ani = animation.FuncAnimation(fig, animate, frames=len(x1),
                              init_func=init, interval=40, blit=True)
plt.show()