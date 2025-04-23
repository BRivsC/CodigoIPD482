% Script del libro de robótica para calcular cinemática directa e inversa a
% partir de las ecuaciones de restricción de ruedas

% Nota: en el libro la matriz de restricciones aparece como B y la de los
% radios de las ruedas como D. Acá fueron cambiados para que se adecúen a
% lo que se vio en clases.


%% Omnidireccional 5 ruedas
clear; close; clc; 
%Creación de variables simbólicas
% r: Radio de las ruedas
% L: Distancia del centro del robot hacia la rueda
% theta: Ángulo de 
syms r L theta;
%Cálculo de la cinemática directa respecto al sistema global
% Ángulos de las ruedas
theta_1 = 0;
theta_2 = 2*pi/5;
theta_3 = 4*pi/5;
theta_4 = 6*pi/5;
theta_5 = 8*pi/5;

% A: Matriz de restricciones para cada rueda. Cada fila corresponde a una
% rueda
A=[ sin(theta_1) -cos(theta_1) -L; 
    sin(theta_2) -cos(theta_2) -L;
    sin(theta_3) -cos(theta_3) -L;
    sin(theta_4) -cos(theta_4) -L;
    sin(theta_5) -cos(theta_5) -L]


% B: Matriz de radios de las ruedas. Cada fila corresponde a una rueda    
B=[r 0 0 0 0;
   0 r 0 0 0;
   0 0 r 0 0;
   0 0 0 r 0;
   0 0 0 0 r] 

% Matriz Jacobiana directa
J=simplify(pinv(A)*B) 

% Matriz de rotación
Rdi=[cos(theta) -sin(theta)   0;
     sin(theta)  cos(theta)   0;
             0            0   1];
Jglobal=simplify(Rdi*J)

% Cálculo de la cinemática inversa respecto al sistema global

invJ=pinv(J); %matriz Jacobiana inversa
Rin=[cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];
invJglobal=simplify(invJ*Rin)

%% Mostrar en consola las matrices obtenidas
clc
A_round = vpa(A,2)
B_round = vpa(B,2)
J_round = vpa(J,2)
Jglobal_round = vpa(expand(Jglobal),2)
invJ_round = vpa(invJ,2)
invJglobal_round = vpa(expand(invJglobal),2)
