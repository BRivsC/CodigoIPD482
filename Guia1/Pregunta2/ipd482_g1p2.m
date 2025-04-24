%% Guía 1 IPD482, pregunta 2
% Pregunta 2: Perfiles de velocidad de omnidireccional de 5 ruedas
% Bastián Rivas
clear; close all; clc;
%% Generación del camino
% Coordenadas de los puntos
% Cada punto del camino está descrito por la posición (x,y) y el ángulo de
% pose respecto al sistema de referencia externo.
% Ojo: Cada uno es un vector columna!

MAX_PTS = 20; %Nro de puntos del camino
x_linea = linspace(0,2,MAX_PTS)'; %Puntos equiespaciados en el eje x
y_linea = zeros(MAX_PTS,1);   % Puntos en Y. Hardcodeado en este caso para ser 0
ang = linspace(0,2*pi,MAX_PTS)'; %Ángulo de pose de cada punto. 
                                 % Pensado en completar 1 vuelta al llegar al final


% Visualizar camino con flecha de pose:
quiver(x_linea,y_linea,cos(ang),sin(ang))
title('Poses del robot en el camino')
axis equal
xlabel('X')
ylabel('Y')

% Matriz de puntos: cada fila es un punto del camino
camino = [x_linea y_linea ang];

%-------------


%% Perfiles de velocidad
% Matriz modelo cinemático directo del robot omnidireccional
% Obtenida desde script modelo_omni_5ruedas.m
% Jglobal =
% 
%[  0.4*r*sin(theta), 0.38*r*cos(theta) + 0.12*r*sin(theta), 0.24*r*cos(theta) - 0.32*r*sin(theta), - 0.24*r*cos(theta) - 0.32*r*sin(theta),   0.12*r*sin(theta) - 0.38*r*cos(theta)]
%[- 0.4*r*cos(theta), 0.38*r*sin(theta) - 0.12*r*cos(theta), 0.32*r*cos(theta) + 0.24*r*sin(theta),   0.32*r*cos(theta) - 0.24*r*sin(theta), - 0.12*r*cos(theta) - 0.38*r*sin(theta)]
%[                               -(0.2*r)/L,                            -(0.2*r)/L,                            -(0.2*r)/L,                              -(0.2*r)/L,                              -(0.2*r)/L]
% 

% theta_N es el ángulo de pose de la rueda N respecto a 
% sistema de referencia externo
% theta_1 = 0 + ang_robot;
% theta_2 = 2*pi/5 + theta_1;
% theta_3 = 4*pi/5 + theta_1;
% theta_4 = 6*pi/5 + theta_1;
% theta_5 = 8*pi/5 + theta_1;

% Parámetros
r = 0.04; % Radio de las ruedas según datos técnicos de Robotino (en metros) 
L = 0.2;  % Distancia del centro del robot hacia la rueda (en metros)
dt = 0.5; % Tiempo de muestreo

% Perfil de velocidades
vel_camino=zeros(5,MAX_PTS+1); % Cada columna corresponde a un instante de tiempo
                               % Cada fila corresponde a una rueda
                               % Se agrega un extra para el instante 0

for k=1:MAX_PTS-1
    
    theta_1 = ang(k); %nuevo ángulo de pose
    theta_2 = 2*pi/5 + theta_1;
    theta_3 = 4*pi/5 + theta_1;
    theta_4 = 6*pi/5 + theta_1;
    theta_5 = 8*pi/5 + theta_1;

    
    %Matriz del modelo omnidireccional con valores del instante k
    Jglobal =[ 0.4*r*sin(ang(k)), 0.38*r*cos(ang(k)) + 0.12*r*sin(ang(k)), 0.24*r*cos(ang(k)) - 0.32*r*sin(ang(k)), -0.24*r*cos(ang(k)) - 0.32*r*sin(ang(k)),  0.12*r*sin(ang(k))-0.38*r*cos(ang(k));
              -0.4*r*cos(ang(k)), 0.38*r*sin(ang(k)) - 0.12*r*cos(ang(k)), 0.32*r*cos(ang(k)) + 0.24*r*sin(ang(k)),  0.32*r*cos(ang(k)) - 0.24*r*sin(ang(k)), -0.12*r*cos(ang(k))-0.38*r*sin(ang(k));
                      -(0.2*r)/L,                              -(0.2*r)/L,                              -(0.2*r)/L,                               -(0.2*r)/L,                             -(0.2*r)/L];
    
    %Obtener velocidad para ir al siguiente punto usando pseudoinversa
    vel_camino(:,k)=pinv(Jglobal)*(transpose(camino(k+1,:)-camino(k,:)))/dt;
    
end

% figure()
 tTotal=0:dt:(length(vel_camino(1,:))-1)*dt;
 plot(tTotal,vel_camino(1,:)/r,'r',tTotal,vel_camino(2,:)/r,'g',tTotal,vel_camino(3,:)/r,'b',tTotal,vel_camino(4,:)/r,'m',tTotal,vel_camino(5,:)/r,'k')
 title('Velocidades angulares a lo largo del trayecto')
 xlabel('Tiempo (s)')
 ylabel('Velocidad angular (rad/s)')
 legend('w1','w2','w3','w4','w5','orientation','horizontal')

 
%% Animación del robot
figure;
hold on;
axis equal;
xlabel('X');
xlim([-0.5 2.5]);
pause(2); % Pausa para que el gráfico no se recorte mientras renderiza. Gracias Matlab
ylabel('Y');
ylim([-1 1]); % Ajustar límites en Y si es necesario
title('Animación de la trayectoria del robot');
% quiver(x_linea, y_linea, cos(ang), sin(ang), 'k--'); % Camino de referencia

flecha_pose = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2); % Inicialización del robot
cuerpo_robot = plot(0, 0, 'b'); % Inicialización del círculo

for k = 1:MAX_PTS
    % Actualizar posición y orientación del robot
    x_pos = camino(k, 1);
    y_pos = camino(k, 2);
    theta = camino(k, 3);
    
    % Actualizar flecha del robot con largo 0.2
    set(flecha_pose, 'XData', x_pos, 'YData', y_pos, ...
        'UData', 0.2 * cos(theta), 'VData', 0.2 * sin(theta));
    
    % Dibujar círculo centrado en el origen de la flecha
    theta_circle = linspace(0, 2 * pi, 100);
    x_circle = 0.2 * cos(theta_circle) + x_pos;
    y_circle = 0.2 * sin(theta_circle) + y_pos;
    set(cuerpo_robot, 'XData', x_circle, 'YData', y_circle);
    exportgraphics(gcf,'testAnimated.gif','Append',true);
    
    % pause(0.4); % Pausa para la animación
end