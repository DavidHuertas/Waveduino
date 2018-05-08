% MI PRIMER FILTRO DE KALMAN

% Tenemos 10 datos de una tensi�n (v) que var�a con el tiempo (t)
% Se trata de aplicar un filtro de Kalman a la informaci�n obtenida. Como
% datos iniciales podemos decir que:
%
%   -No tenemos se�al de control: u_k=0
%   -A=1, H=1, x_0=0
%   -P_0=1  (Si fuese 0 estar�a suponiendo que en el inicio no habr�a
%   error)
% 

clear all
close all

%1) DATOS EXPERIMENTALES

for i=1:10
t(i)=i;                                                     %Tiempo
end

v=[0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45];      %Tensi�n

%2) INICIALIZACI�N DE VARIABLES:

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

%3) IMPLEMENTACI�N DEL FILTRO DE KALMAN:

for i=2:10

%PREDICCI�N
    x_(i)=x(i-1);
    P_(i)=P(i-1);
%CORRECCI�N
    K(i)=P_(i)/(P_(i)+R);
    x(i)=x_(i)+K(i)*(z(i)-x_(i));
    P(i)=(1-K(i))*P_(i);
    
end

figure(1);
subplot(3,1,1);hold on
plot(t,v,'r','Marker','x');
xlabel('Tiempo');
ylabel('Tensi�n Experimental');
subplot(3,1,2);hold on
plot(t,x,'b','Marker','o');
xlabel('Tiempo');
ylabel('Tensi�n Filtrada');
subplot(3,1,3);hold on
plot(t,v,'r','Marker','x');
plot(t,x,'b','Marker','o');
xlabel('Tiempo');
ylabel('Tensi�n');


