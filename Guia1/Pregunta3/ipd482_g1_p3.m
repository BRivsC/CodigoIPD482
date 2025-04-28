%% Guía 1 IPD482
% Pregunta 3: Modelar y simular un tractor con 2 trailers (unicycle + 2 semirremolques)
% Bastián Rivas

clear; clc; close all;

%% Parámetros geométricos
L0 = 0.5;    % distancia eje tractor a punto de enganche (m)
             % Basada en Clearpath Jackal UGV
L1 = 0.66;   % longitud trailer 1 (m)
L2 = 0.66;   % longitud trailer 2 (m)
             % Basados en la mitad de un Gorilla Cart 170L Steel Mesh
Lh1 = 0.2;   % offset enganche trailer 1 (m) (+ off-axle)
Lh2 = 0.2;   % offset enganche trailer 2 (m)

%% Tiempo de simulación
T  = 25;     % [s]
dt = 0.01;   % paso de integración
t  = 0:dt:T;

%% Señales de control para la línea
% Nota: Ejecutar solo UNA de las celdas de señales de control para escoger
% un camino!
% Ejecutar: Clickear en celda a correr y pulsar Ctrl + Enter
% Alternativamente se puede comentar una celda y correr con el botón Run
v0     = 0.5 * ones(size(t));            % 0.5 m/s constante
omega0 = zeros(size(t));                 % Línea recta: omega0 = 0


%% Señales para trazar un círculo:
%R = 2;                          % radio del círculo deseado [m]
%v0 = 0.5 * ones(size(t));       % velocidad constante [m/s]
%omega0 = v0 / R;                % velocidad angular constante [rad/s]


%% Estado del robot: q = [x2; y2; th2; th1; th0]
Q = zeros(5,length(t));
% condiciones iniciales
Q(:,1) = [0; 0;  0;  0;  0];

%% Simulación por Euler directo
for k = 1:length(t)-1
    % extrae estado
    x2  = Q(1,k); y2 = Q(2,k);
    th2 = Q(3,k); th1 = Q(4,k); th0 = Q(5,k);
    b1 = th0-th1;  b2 = th1-th2;
    
    % construye J1, J2
    J1 = [ -Lh1/L1*cos(b1),  (1/L1)*sin(b1);
            Lh1*sin(b1),      cos(b1)      ];
    J2 = [ -Lh2/L2*cos(b2),  (1/L2)*sin(b2);
            Lh2*sin(b2),      cos(b2)      ];
    
    % S_T2 = G(th2)*J2*J1
    G = [0, cos(th2);
         0, sin(th2);
         1,       0];
    ST2 = G*(J2*J1);
    
    % Matrices S
    c = [1 0];
    S4 = c*J1;
    S5 = c;
    
    % matriz completa S(q)
    S = [ ST2;
          S4;
          S5 ];
    
    % entrada en instante k
    u0 = [ omega0(k); v0(k) ];
    
    % deriva
    qdot = S * u0;
    
    % Euler
    Q(:,k+1) = Q(:,k) + qdot*dt;
end

%% Reconstruir posiciones de tractor (p0), trailer1 (p1), trailer2 (p2)
% Formula: p_i = p_{i-1} - [ L_i*cos(th_i) + Lh_i*cos(th_{i-1});
%                            L_i*sin(th_i) + Lh_i*sin(th_{i-1}) ]
P0 = nan(2,length(t));
P1 = nan(2,length(t));
P2 = nan(2,length(t));
% asumimos tractor midpoint en p0
P0(:,1) = [ Q(1,1) ; Q(2,1) ] + ... 
          [ (L1+Lh1)*cos(Q(5,1)); (L1+Lh1)*sin(Q(5,1)) ];  
% en realidad x2,y2 es p2; calculamos hacia atrás:
for k=1:length(t)
  x2 = Q(1,k); y2 = Q(2,k);
  th2= Q(3,k); th1 = Q(4,k); th0 = Q(5,k);
  P2(:,k) = [x2;y2];
  % trailer1
  P1(:,k) = P2(:,k) + [ L2*cos(th2)+Lh2*cos(th1);
                        L2*sin(th2)+Lh2*sin(th1) ];
  % tractor
  P0(:,k) = P1(:,k) + [ L1*cos(th1)+Lh1*cos(th0);
                        L1*sin(th1)+Lh1*sin(th0) ];
end

%% Dibujo estático de la trayectoria
figure; axis equal; grid on;
hold on;
plot(P0(1,:),P0(2,:),'r-','LineWidth',1.5);    % tractor
plot(P1(1,:),P1(2,:),'b-','LineWidth',1.2);    % trailer1
plot(P2(1,:),P2(2,:),'k-','LineWidth',1.0);    % trailer2
legend('tractor','trailer 1','trailer 2','Location','best');
xlabel('X[m]'); ylabel('Y[m]');
title('Trayectoria del conjunto tractor + 2 trailers');


%% Animación del robot con rectángulos orientados
figure;
hold on;
axis equal;
grid on;
axis([min(P0(1,:))-4 max(P0(1,:))+2 min(P0(2,:))-2 max(P0(2,:))+2]);
pause(2); % Pausa para evitar recorte al renderizar
title('Trayectoria en línea recta');

% Tamaño de cada segmento [largo, ancho]
tractor_size  = [0.508, 0.430]; % largo x ancho (m)
trailer1_size = [0.66, 0.6];
trailer2_size = [0.66, 0.6];

% Crear "patches" (rectángulos) para cada segmento
tractor_patch  = fill(NaN,NaN,'r');
trailer1_patch = fill(NaN,NaN,'b');
trailer2_patch = fill(NaN,NaN,'y');

for k=1:10:length(t)
    % Extrae orientación y posición
    pos0 = P0(:,k); th0 = Q(5,k);
    pos1 = P1(:,k); th1 = Q(4,k);
    pos2 = P2(:,k); th2 = Q(3,k);
    
    % Calcular vértices de cada patch con función auxiliar
    [x0, y0] = rect_patch(pos0, th0, tractor_size);
    [x1, y1] = rect_patch(pos1, th1, trailer1_size);
    [x2, y2] = rect_patch(pos2, th2, trailer2_size);
    
    % Actualizar patches
    set(tractor_patch,  'XData', x0, 'YData', y0);
    set(trailer1_patch, 'XData', x1, 'YData', y1);
    set(trailer2_patch, 'XData', x2, 'YData', y2);
    
    exportgraphics(gcf,'NTrailer_linea.gif','Append',true);
end

%%
function [xv, yv] = rect_patch(center, theta, size)
% center = [x; y], theta en radianes, size = [largo ancho]
L = size(1); W = size(2);
% Coordenadas del rectángulo centrado en (0,0)
x_corners = [-L/2 L/2 L/2 -L/2];
y_corners = [-W/2 -W/2 W/2 W/2];

% Rotación
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
coords = R * [x_corners; y_corners];

% Traslación
xv = coords(1,:) + center(1);
yv = coords(2,:) + center(2);
end
