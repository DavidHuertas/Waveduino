%tercer Kalman

%1) DATOS EXPERIMENTALES

n=length(gz);

for i=1:n
t(i)=i;                                                     %Tiempo
end

v=gz';                                                      %Omega

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
R=0.03;
K(1)=P_(1)/(P_(1)+R);
x(1)=x_(1)+K(1)*(z(1)-x_(1));
P(1)=(1-K(1))*P_(1);
Q=0.003;

%3) IMPLEMENTACIÓN DEL FILTRO DE KALMAN:

for i=2:n

%PREDICCIÓN
    x_(i)=x(i-1);
    P_(i)=P(i-1)+Q;
%CORRECCIÓN
    K(i)=P_(i)/(P_(i)+R);
    x(i)=x_(i)+K(i)*(z(i)-x_(i));
    P(i)=(1-K(i))*P_(i);
    
end

figure(1);
subplot(3,1,1);hold on
plot(t,v,'r');
xlabel('Tiempo');
ylabel('Tensión Experimental');
subplot(3,1,2);hold on
plot(t,x,'b');
xlabel('Tiempo');
ylabel('Tensión Filtrada');
subplot(3,1,3);hold on
plot(t,v,'r');
plot(t,x,'b');
xlabel('Tiempo');
ylabel('Tensión');
