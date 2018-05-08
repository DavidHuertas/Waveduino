limpiar
cerrarSerial
abrirSerial

% LECTURA DEL IMU MÁS EFICIENTE:
n=500;
i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

tic
disp('Recibiendo datos')

while i<n
    
    imu=fgetl(s);
    if (imu(1)=='k')
        ax(i)=str2double(fgetl(s));
        ay(i)=str2double(fgetl(s));
        az(i)=str2double(fgetl(s));
        gx(i)=str2double(fgetl(s));
        gy(i)=str2double(fgetl(s));
        gz(i)=str2double(fgetl(s));
        i=i+1;
    end

end
t=toc;
disp('Finalizada la recogida de datos')
dt=t/n;

graw=norm([mean(ax(1:10)) mean(ay(1:10)) mean(az(1:10))]);

gain(1)=9.81/graw;
gain(2)=1/14.375;

offsetax=mean(ax(1:10))*gain(1);
offsetay=mean(ay(1:10))*gain(1);
offsetaz=mean(az(1:10))*gain(1);
offsetaz=offsetaz+norm([offsetax,offsetay,offsetaz]);

ax=ax*gain(1)-offsetax;
ay=ay*gain(1)-offsetay;
az=az*gain(1)-offsetaz;
gx=(gx-mean(gx(1:10)))*gain(2);
gy=(gy-mean(gy(1:10)))*gain(2);
gz=(gz-mean(gz(1:10)))*gain(2);

matr=[ax ay az gx gy gz]';

%%%%%KALMAN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1) DATOS EXPERIMENTALES

a_ximu=matr(1,:);
a_yimu=matr(2,:);
om_zimu=matr(6,:);


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
R=0.03*eye(3);

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

for i=2:n
    theta(i)=theta(i-1)+dt*om_zfilt(i);
end

for i=2:n
    Rot=[cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
    a(:,i)=Rot*a_filt(:,i);
end

for i=2:n
    v(:,i)=v(:,i-1)+dt.*a(:,i);
end

for i=2:n
    x(:,i)=x(:,i-1)+dt.*v(:,i-1);    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



figure(1)
plot(x(1,:),x(2,:));

figure(2)
subplot(5,1,1)
plot(1:n,a_ximu,'r');hold on; %aceleración x imu en rojo
plot(1:n,a_xfilt);            %aceleración x filtrada
subplot(5,1,2)
plot(1:n,a_yimu,'r');hold on; %aceleración y imu en rojo
plot(1:n,a_yfilt);            %aceleración y filtrada
subplot(5,1,3)
plot(1:n,a(1,:));             %aceleración x filtrada absoluta
subplot(5,1,4)
plot(1:n,a(2,:));             %aceleración z filtrada absoluta
subplot(5,1,5)
plot(1:n,om_zimu,'r');hold on;%Vel. giro z imu en rojo
plot(1:n,om_zfilt);           %Vel. giro z filtrada

figure (3)
subplot(2,1,1)
plot(1:n,v(1,:));             %Vel. x
subplot(2,1,2)
plot(1:n,v(2,:));             %Vel. y