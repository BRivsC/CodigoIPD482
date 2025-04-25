% Estado del sistema:
% q = [x2; y2; theta2; theta1; theta0]

% Velocidad segmento 0:
% u0 = [omega0; v0]

% Distancias desde eje hacia punto que tira
% L = [L1; L2]
% Lh = [Lh1; Lh2]

% Ángulos de los segmentos
theta2 = q(3);
theta1 = q(4);
theta0 = q(5);

% Ángulos articulares
beta1 = theta0 - theta1;
beta2 = theta1 - theta2;

% Cinemática uniciclo
G2 = [0, cos(theta2);
      0, sin(theta2);
      1,      0      ];

% J2: off-axle -> usar fórmula exacta
if Lh(2) ~= 0
    J2 = [ -Lh(2)/L(2)*cos(beta2), cos(beta2)/L(2);
            Lh(2)/L(2)*sin(beta2), sin(beta2)/L(2);
            Lh(2)*sin(beta2),      cos(beta2)];
else
    % on-axle approximation para J2
    % se usa el método aproximado del paper (Eqs. 14–16)
    % Asignar valores realistas si se necesita calcularlos explícitamente
    % Aquí se omite y solo usamos el input directo
    error('On-axle approximation for J2 not implemented here.');
end

% J1: off-axle
if Lh(1) ~= 0
    J1 = [ -Lh(1)/L(1)*cos(beta1), cos(beta1)/L(1);
            Lh(1)/L(1)*sin(beta1), sin(beta1)/L(1);
            Lh(1)*sin(beta1),      cos(beta1)];
else
    error('On-axle approximation for J1 not implemented here.');
end

% Propagación de velocidades
u1 = J1 * u0;
u2 = J2 * u1;

% Cinemática del guidance segment (trailer 2)
dq = G2 * u2;

 
% Estado inicial
q = [0; 0; pi/2; pi/2; pi/2];  % Posición inicial alineada

% Input del tractor (v, omega)
u0 = [0.1; -0.5];  % Giro hacia atrás

% Dimensiones del sistema
L = [1.0; 0.8];
Lh = [0.3; 0.0];  % Segundo trailer on-axle

% Evaluar derivada del estado
dq = g2t_kinematics(q, u0, L, Lh);
disp(dq)
