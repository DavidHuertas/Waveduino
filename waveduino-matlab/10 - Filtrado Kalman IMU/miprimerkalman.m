% MI PRIMER FILTRO DE KALMAN

% Tenemos 10 datos de una tensión (v) que varía con el tiempo (t)
% Se trata de aplicar un filtro de Kalman a la información obtenida. Como
% datos iniciales podemos decir que:
%
%   -No tenemos señal de control: u_k=0
%   -A=1, H=1, x_0=0
%   -P_0=1  (Si fuese 0 estaría suponiendo que en el inicio no habría
%   error)
% 

clear all
close all

%1) DATOS EXPERIMENTALES

for i=1:10
t(i)=i;                                                     %Tiempo
end

v=[0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45];      %Tensión

%2) INICIALIZACIÓN DE VARIABLES:

x_=zeros(1,10);
x=zeros(1,10);
P_=zeros(1,10);
P=zeros(1,10);
K=zeros(1,10);
A=1;
H=1;
z=v;
%Primer paso (aparte):
x_(1)=0;
P_(1)=1;
R=0.1;
K(1)=P_(1)/(P_(1)+R);
x(1)=x_(1)+K(1)*(z(1)-x_(1));
P(1)=(1-K(1))*P_(1);

%3) IMPLEMENTACIÓN DEL FILTRO DE KALMAN:

for i=2:10

%PREDICCIÓN
    x_(i)=x(i-1);
    P_(i)=P(i-1);
%CORRECCIÓN
    K(i)=P_(i)/(P_(i)+R);
    x(i)=x_(i)+K(i)*(z(i)-x_(i));
    P(i)=(1-K(i))*P_(i);
    
end

figure(1);
subplot(3,1,1);hold on
plot(t,v,'r','Marker','x');
xlabel('Tiempo');
ylabel('Tensión Experimental');
subplot(3,1,2);hold on
plot(t,x,'b','Marker','o');
xlabel('Tiempo');
ylabel('Tensión Filtrada');
subplot(3,1,3);hold on
plot(t,v,'r','Marker','x');
plot(t,x,'b','Marker','o');
xlabel('Tiempo');
ylabel('Tensión');


