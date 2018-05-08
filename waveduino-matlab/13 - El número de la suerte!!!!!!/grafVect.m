limpiar
cerrarSerial
abrirSerial

% LECTURA DEL IMU MÁS EFICIENTE:
n=1000;
i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

gain(1)=1/(3.3*8);
gain(2)=1/14.375;

disp('Calibrando')

while i<20
    
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
dt=t/n;

offsetax=mean(ax(5:20))*gain(1);
offsetay=mean(ay(5:20))*gain(1);
offsetaz=mean(az(5:20))*gain(1);
offsetaz=offsetaz+norm([offsetax,offsetay,offsetaz]);

disp('Calibrado Finalizado')

disp('Recogiendo Datos')
tic
i=1;

theta=zeros(1,n);
%Datos KALMAN
Q=0.003;
R=0.03;
x_=zeros(1,n);
x=zeros(1,n);
P_=zeros(1,n);
P=zeros(1,n);
K=zeros(1,n);
A=1;
H=1;
z=gz;
%Primer paso Kalman
x_(1)=0;
P_(1)=1;
K(1)=P_(1)/(P_(1)+R);
x(1)=x_(1)+K(1)*(z(1)-x_(1));
P(1)=(1-K(1))*P_(1);

j=1;
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
    
    t=toc;
    dt=t/i;    
    
    ax(i)=ax(i)*gain(1)-offsetax;
    ay(i)=ay(i)*gain(1)-offsetay;
    az(i)=az(i)*gain(1)-offsetaz;
    gx(i)=(gx(i)-mean(gx(1:10)))*gain(2);
    gy(i)=(gy(i)-mean(gy(1:10)))*gain(2);
    gz(i)=(gz(i)-mean(gz(1:10)))*gain(2);

    
    if (i==j)
        %FILTRO DE KALMAN
        %PREDICCIÓN
        x_(i)=x(i-1);
        P_(i)=P(i-1)+Q;
        %CORRECCIÓN
        K(i)=P_(i)/(P_(i)+R);
        x(i)=x_(i)+K(i)*(z(i)-x_(i));
        P(i)=(1-K(i))*P_(i);
        %INTEGRACION EULER
        theta(i)=theta(i-1)+dt*x(i)-30/1000;
        j=j+1;
    end
    
    %Forzamos a Dibujar el punto de una circunferencia de radio 1:
    cla;
    line([0, cos(theta(i)*pi/180)], [0, sin(theta(i)*pi/180)], 'Color',...
        'g', 'LineWidth', 3, 'Marker', 'o');
    axis([-2 2 -2 2]);
    axis square;
    grid on;
    xlabel ('GIROSCOPIO EN Z');
    drawnow;
    
end