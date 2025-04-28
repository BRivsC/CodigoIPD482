%% Guía 1 IPD482
% Pregunta 4: Simulación dinámica de tractor skid-steer 4R con fricción 
%             modelada con LuGre
% Bastián Rivas

clear; clc; close all;

%% Parámetros del chasis
m   = 50;        % masa [kg]
I   = 2.0;       % inercia [kg·m^2]
r   = 0.15;      % radio de rueda [m]
c   = 0.3;       % semiancho chassis [m]
x0  = 0.1;       % ICR operativo [m]

%% Parámetros LuGre (idénticos en cada rueda)
% Obtenidos de la tabla 4 del paper
sigma0 = 20;     % rigidez [N/m]
sigma1 = 5;      % amortiguamiento interno [N·s/m]
sigma2 = 20;     % fricción viscosa externa [N·s/m]
Fc      = 0.28;  % Coulomb cinética [N]
Fs      = 0.34;  % Coulomb estática [N]
vs      = 12.5;  % velocidad Stribeck [m/s]

%% Tiempo de simulación
tspan = [0 5];
% Estado inicial: [X;Y;theta; vx;omega; z1..z4]
q = [0; 0; 0; 0; 0; zeros(4,1)];

%% Perfil de torques
%tauL = @(t) 50*(t<5) + 20*(t>=5);
%tauR = @(t) 30*sin(0.5*t) + 40;
tauL = @(t) 0;
tauR = @(t) 0.3;
%% Integrar
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[T,Y] = ode45(@(t,y) dynamics(t,y), tspan, q, opts);

%% Plots
figure;
plot(Y(:,1),Y(:,2),'LineWidth',1.5); axis equal; grid on
xlabel('X [m]'); ylabel('Y [m]');
title('Trayectoria del tractor skid-steer');

figure;
subplot(2,1,1), plot(T,Y(:,4)), ylabel('v_x [m/s]'), grid on
subplot(2,1,2), plot(T,Y(:,5)), ylabel('\omega [rad/s]'), xlabel('t [s]'), grid on

%% --- función de dinámica ---
function dy = dynamics(t,y)
  % parámetros en workspace
  m   = evalin('base','m');  I = evalin('base','I');
  r   = evalin('base','r');  c = evalin('base','c'); x0 = evalin('base','x0');
  sigma0=evalin('base','sigma0'); sigma1=evalin('base','sigma1');
  sigma2=evalin('base','sigma2'); Fc=evalin('base','Fc'); Fs=evalin('base','Fs'); vs=evalin('base','vs');
  tauL = evalin('base','tauL'); tauR=evalin('base','tauR');
  
  % estados
  th=y(3);
  vx=y(4); w=y(5);
  z=y(6:9);
  
  % velocidades rueda izq/der (longitudinal)
  vL = vx - c*w;
  vR = vx + c*w;
  v_rel = [-vL; -vL; -vR; -vR];  % 4 ruedas
  
  % LuGre en cada rueda
  dz=zeros(4,1); Ff=zeros(4,1);
  for i=1:4
    v=v_rel(i);
    g = Fc + (Fs-Fc)*exp(-(v/vs)^2);
    dz(i) = v - abs(v)/g * z(i);
    Ff(i) = sigma0*z(i) + sigma1*dz(i) + sigma2*v;
  end
  
  % Fuerzas resistivas en chasis
  Frx = sum(Ff);
  Mr  = c*(Ff(1)+Ff(2)-Ff(3)-Ff(4));
  
  % dinámica (vx,ω)
  Mbar = [m 0; 0 m*x0^2+I];
  Tau  = [tauL(t)+tauR(t); -c*tauL(t)+c*tauR(t)]/r;
  Rbar = [Frx; Mr];
  accel = Mbar \ (Tau - Rbar);
  vx_dot = accel(1);
  w_dot  = accel(2);
  
  % cinematica global
  vy = -x0*w;
  X_dot = vx*cos(th) - vy*sin(th);
  Y_dot = vx*sin(th) + vy*cos(th);
  th_dot = w;
  
  dy = [X_dot; Y_dot; th_dot; vx_dot; w_dot; dz];
end
