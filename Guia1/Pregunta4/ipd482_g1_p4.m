% Parámetros del robot
m = 1;         % Masa del robot [kg]
I = 0.0036;    % Momento de inercia [kg*m^2]
r = 0.0265;    % Radio efectivo de las ruedas [m]
a = 0.039;     % Distancia hacia las ruedas delanteras/traseras [m]
b = 0.034;     % Distancia hacia las ruedas laterales [m]

% Tiempo de simulación
dt = 0.01;     % Paso de tiempo [s]
T = 10;        % Tiempo total de simulación [s]
N = T/dt;      % Número de pasos

% Inicialización de estados
X = 0; Y = 0; theta = 0;          % Posición y orientación inicial
vx = 0; vy = 0; omega = 0;        % Velocidades iniciales

% Matrices de registro
traj = zeros(N, 3);  % Trayectoria (X, Y, theta)

% Perfil de torque
perfil = @(t) [0.5*sin(2*pi*0.1*t), 0.5*sin(2*pi*0.1*t), -0.5*sin(2*pi*0.1*t), -0.5*sin(2*pi*0.1*t)];

% Simulación
for i = 1:N
    t = (i-1)*dt;
    T_wheels = perfil(t);  % Torque de las ruedas

    % Dinámica de las fuerzas
    Fx = (T_wheels(1) + T_wheels(2) + T_wheels(3) + T_wheels(4)) / r; % Fuerza neta en x
    Fy = 0; % Sin movimiento lateral
    Mz = (T_wheels(2) + T_wheels(3) - T_wheels(1) - T_wheels(4)) * a; % Momento neto

    % Actualización de velocidades
    vx = vx + (Fx/m)*dt;
    omega = omega + (Mz/I)*dt;

    % Actualización de posición y orientación
    X = X + vx * cos(theta) * dt;
    Y = Y + vx * sin(theta) * dt;
    theta = theta + omega * dt;

    % Registro
    traj(i, :) = [X, Y, theta];
end

% Graficar resultados
figure;
subplot(2,1,1);
plot(traj(:,1), traj(:,2));
xlabel('Posición X [m]');
ylabel('Posición Y [m]');
title('Trayectoria del robot');

subplot(2,1,2);
plot((0:dt:T-dt), traj(:,3));
xlabel('Tiempo [s]');
ylabel('Orientación θ [rad]');
title('Orientación del robot');