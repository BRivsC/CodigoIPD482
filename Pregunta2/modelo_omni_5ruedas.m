% Script del libro de robótica para calcular cinemática directa e inversa a
% partir de las ecuaciones de restricción de ruedas

% Nota: en el libro la matriz de restricciones aparece como B y la de los
% radios de las ruedas como D. Acá fueron cambiados para que se adecúen a
% lo que se vio en clases.

%% Omnidireccional 3 ruedas
clear; close; clc; 
%Creación de variables simbólicas
syms r L theta
%Cálculo de la cinemática directa respecto al sistema global
A=[sqrt(3)/2 -1/2 -L; 0 1 -L; -sqrt(3)/2 -1/2 -L]
B=[r 0 0; 0 r 0;0 0 r]
J=simplify(inv(A)*B) %matriz Jacobiana directa
Rdi=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
Jglobal=simplify(Rdi*J)
%Cáculo de la cinemática inversa respecto al sistema global
invJ=inv(J); %matriz Jacobiana inversa
Rin=[cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];
invJglobal=simplify(invJ*Rin)

%% Lo mío: Omnidireccional 5 ruedas
clear; close; clc; 
%Creación de variables simbólicas
syms r L theta
%Cálculo de la cinemática directa respecto al sistema global
% A: Matriz de restricciones para cada rueda. Cada fila corresponde a una
% rueda
A=[ sin(0) -1/2 -L; 
   0             1 -L; 
   -sqrt(3)/2 -1/2 -L]

B=[r 0 0; 0 r 0;0 0 r]
J=simplify(inv(A)*B) %matriz Jacobiana directa
Rdi=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
Jglobal=simplify(Rdi*J)
%Cáculo de la cinemática inversa respecto al sistema global
invJ=inv(J); %matriz Jacobiana inversa
Rin=[cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];
invJglobal=simplify(invJ*Rin)