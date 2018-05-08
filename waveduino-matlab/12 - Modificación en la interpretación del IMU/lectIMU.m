limpiar
cerrarSerial
abrirSerial

% LECTURA DEL IMU MÁS EFICIENTE:
n=3000;
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

offsetax=mean(ax(1:10))*gain(1);
offsetay=mean(ay(1:10))*gain(1);
offsetaz=mean(az(1:10))*gain(1);
offsetaz=offsetaz+norm([offsetax,offsetay,offsetaz]);

ax=ax*gain(1)-offsetax;
ay=ay*gain(1)-offsetay;
az=az*gain(1)-offsetaz;
gx=(gx-mean(gx(1:10)))*gain(2);
gy=(gy-mean(gy(1:10)))*gain(2);
for i=1:n
gz(i)=(gz(i)-mean(gz(1:10)))*gain(2);%-0.03*i^(1/2);
end

matr=[ax ay az gx gy gz]';

kalman
giro

figure(3)
for i=1:n
    %Forzamos a Dibujar el punto de una circunferencia de radio 1:
    cla;
    line([0, cos(theta(i)*pi/180)], [0, sin(theta(i)*pi/180)], 'Color',...
        'g', 'LineWidth', 3, 'Marker', 'o');
    axis([-2 2 -2 2]);
    axis square;
    grid on;
    xlabel ('GIROSCOPIO EN Z');
    drawnow;
    pause(dt);
end