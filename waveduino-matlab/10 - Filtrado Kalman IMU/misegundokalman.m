% MI SEGUNDO FILTRO DE KALMAN

% Tenemos n datos de dos aceleraciones (a_ximu, a_yimu) y de una velocidad
% angular (omzimu) que varían con el tiempo.
% Se trata de aplicar un filtro de Kalman a la información obtenida. Como
% datos iniciales podemos decir que:
%
%   -No tenemos señal de control: u_k=0
%   -A=eye(3), H=eye(3), x_0=0
%   -P_0=1  (Si fuese 0 estaría suponiendo que en el inicio no habría
%   error)
% 


%1) DATOS EXPERIMENTALES

a_ximu=valuesmatrix(1,:);
a_yimu=valuesmatrix(2,:);
om_zimu=valuesmatrix(6,:);

n=length(a_ximu);
dt=10/n;

%2) INICIALIZACIÓN DE VARIABLES:

x_=zeros(3,n);
x=zeros(3,n);
P_=eye(3);
P=eye(3);
K=zeros(3);
A=eye(3);
H=eye(3);
z=[a_ximu' a_yimu' om_zimu'];
z=z';
Q=[0.001 0 0; 0 0.001 0; 0 0 0.003];
R=0.01*eye(3);

%Primer paso (aparte):


%3) IMPLEMENTACIÓN DEL FILTRO DE KALMAN:

for i=2:n

%PREDICCIÓN
    x_(:,i)=A*x(:,i-1);
    P_=A*P*A'+Q;
%CORRECCIÓN
    K=P_*H'*((H*P_*H'+R)^(-1));
    x(:,i)=x_(:,i)+K*(z(:,i)-H*x_(:,i));
    P=(eye(3)-K*H)*P_;
    
end

a_xfilt=x(1,:);
a_yfilt=x(2,:);

a_filt=[a_xfilt;a_yfilt];

om_zfilt=x(3,:);

v=zeros(2,n);
a=zeros(2,n);
x=zeros(2,n);

%4) INTEGRACIÓN DE LOS RESULTADOS:

theta=zeros(1,n);

for i=2,n;
    theta(i)=theta(i-1)+dt*om_zfilt(i);
end

for i=2,n;
    Rot=[cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
    a(:,i)=Rot*a_filt(:,i);
end

for i=2,n;
    v(:,i)=v(:,i-1)+dt.*a(:,i);
end

for i=2,n;
    x(:,i)=x(:,i-1)+dt.*v(:,i-1);    
end

figure(1)
plot(x(1,:),x(2,:));

figure(2)
subplot(3,1,1)
plot(1:n,a_ximu,'r');hold on;
plot(1:n,a_xfilt);
subplot(3,1,2)
plot(1:n,a_yimu,'r');hold on;
plot(1:n,a_yfilt);
subplot(3,1,3)
plot(1:n,om_zimu,'r');hold on;
plot(1:n,om_zfilt);