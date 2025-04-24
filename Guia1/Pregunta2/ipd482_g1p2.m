%% Guía 1 IPD482, pregunta 2
% Pregunta 2: Perfiles de velocidad de omnidireccional de 5 ruedas
% Bastián Rivas

%% Generación del camino
% Línea recta

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
% theta_2 = 2*pi/5 + ang_robot;
% theta_3 = 4*pi/5 + ang_robot;
% theta_4 = 6*pi/5 + ang_robot;
% theta_5 = 8*pi/5 + ang_robot;

%Velocidad angular (señal de control): wn= vn/r,
%Radio (en metros) de las ruedas según datos técnicos de Robotino:
r=0.04;
L=0.2; %Distancia del centro del robot hacia la rueda (en metros)


% Perfil de velocidades
vCuad=zeros(5,21); % Cada columna corresponde a un instante de tiempo
                   % Cada fila corresponde a una rueda

for k=1:20
    
    theta_1=pi/3+ang(k); %nuevo ángulo de pose
    theta_2=2*pi/3+theta_1;
    theta_3=4*pi/3+theta_1;
    
    %Matriz del modelo omnidireccional con valores del instante k
    A= 2/3*[-sin(theta_1),-sin(theta_1+theta_2),-sin(theta_1+theta_3);
        cos(theta_1),cos(theta_1+theta_2),cos(theta_1+theta_3);
        1/(2*R),1/(2*R),1/(2*R)];
    
    %Obtener velocidad para ir al siguiente punto usando pseudoinversa
    vCuad(:,k)=pinv(A)*(transpose(cuadrado(k+1,:)-cuadrado(k,:)))/dt;
    
end

% figure()
 tTotal=0:dt:(length(vCuad(1,:))-1)*dt;
 plot(tTotal,vCuad(1,:)/r,'r',tTotal,vCuad(2,:)/r,'g',tTotal,vCuad(3,:)/r,'b')
 title('Velocidades angulares para el camino cuadrado')
 xlabel('Tiempo (s)')
 ylabel('Velocidad angular (rad/s)')
 legend('w1','w2','w3')

%c) 8-invertido

vOcho=zeros(3,21);

for k=1:20
    
    theta_1=pi/3+angOcho(k); %nuevo ángulo de pose
    theta_2=2*pi/3+theta_1;
    theta_3=4*pi/3+theta_1;
    
    %Matriz del modelo omnidireccional con valores del instante k
    A= 2/3*[-sin(theta_1),-sin(theta_1+theta_2),-sin(theta_1+theta_3);
        cos(theta_1),cos(theta_1+theta_2),cos(theta_1+theta_3);
        1/(2*R),1/(2*R),1/(2*R)];
    
    %Obtener velocidad para ir al siguiente punto usando pseudoinversa
vOcho(:,k)=pinv(A)*(transpose(ochoInvertido(k+1,:)-ochoInvertido(k,:)))/dt;
    
end

% figure()
% tTotal=0:dt:(length(vOcho(1,:))-1)*dt;
% plot(tTotal,vOcho(1,:)/r,'r',tTotal,vOcho(2,:)/r,'g',
% tTotal,vOcho(3,:)/r,'b')
% title('Velocidades angulares para el camino ocho invertido')
% xlabel('Tiempo (s)')
% ylabel('Velocidad angular (rad/s)')
% legend('w1','w2','w3')



