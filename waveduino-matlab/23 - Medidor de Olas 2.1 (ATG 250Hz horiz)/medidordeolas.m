%% PASOS PREVIOS

addpath('Quaternions');	%Incluye librería de cuaterniones.

%% ABRIR EL PUERTO SERIE

puerto = 'COM3';
s = serial (puerto,'DataBits',8,'StopBits',1,'BaudRate',115200,'Parity',...
    'none');
fopen(s);
disp('ABIERTO EL PUERTO SERIE EN "s"')

%% LECTURA DE DATOS

n=30000;
t=30;

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
temp=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

disp('Recibiendo datos en')
disp('3')
pause(1)
disp('2')
pause(1)
disp('1')
pause(1)
disp('Recibiendo...')
tic

while (i<=n && toc<=t)
    
    imu=fgetl(s);
    if (imu(1)=='k')
        ax(i)=str2double(fgetl(s));
        ay(i)=str2double(fgetl(s));
        az(i)=str2double(fgetl(s));
        temp(i)=str2double(fgetl(s));
        gx(i)=str2double(fgetl(s));
        gy(i)=str2double(fgetl(s));
        gz(i)=str2double(fgetl(s));
        i=i+1;
    end

end

disp('Finalizada la recogida de datos')

if i==n
    t=toc;
else
    ax=ax(1:i-1);
    ay=ay(1:i-1);
    az=az(1:i-1);
    gx=gx(1:i-1);
    gy=gy(1:i-1);
    gz=gz(1:i-1);
    temp=temp(1:i-1);
    n=i-1;
end

%% CERRAR TODOS LOS PUERTOS SERIE

%Este script se encarga de cerrar y limpiar todas las comunicaciones con
%todos los puertos serie abiertos, antes del procesado de datos

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all
disp ('EL PUERTO SERIE HA SIDO CERRADO')

%% APLICAR GANANCIAS Y OFFSETS, CÁLCULO DE FRECUENCIA DE MUESTREO

dt=t/n;

temp=35-(-13200-temp)./280;

gain_gyr=1./(14.375+((15.3733-14.375)/(32-25)).*(temp-25));  

ax=ax*gain_ac;
ay=ay*gain_ac;
az=az*gain_ac;
gx=(gx-offset_gyr(1)).*gain_gyr;
gy=(gy-offset_gyr(2)).*gain_gyr;
gz=(gz-offset_gyr(3)).*gain_gyr;

%Eliminar picos en giroscopio:

for i=1:n-1
    if(abs(gx(i+1)-gx(i))>500)
        gx(i+1)=gx(i);
    end
    if(abs(gy(i+1)-gy(i))>500)
        gy(i+1)=gy(i);
    end
    if(abs(gz(i+1)-gz(i))>500)
        gz(i+1)=gz(i);
    end
end
    

%Eliminar picos en acelerómetro:

for i=1:n-1
    if(abs(ax(i+1)-ax(i))>500)
        gx(i+1)=gx(i);
    end
    if(abs(ay(i+1)-ay(i))>500)
        ay(i+1)=ay(i);
    end
    if(abs(az(i+1)-az(i))>20)
        az(i+1)=az(i);
    end
end
    
    tiempo = linspace(1,t,n);

%% DETECTAR ACELERACIONES NULAS

% Busca las aceleraciones cercanas al cero (dentro del umbral de
% aceleración nula), y crea una variable lógica que avisa de si el muestreo
% está o no en la zona del umbral

% Cálculo del módulo de la aceleración
ac_mod = sqrt(ax.*ax + ay.*ay + az.*az);

% Filtro Paso Alto a la aceleración
frecCorte = 0.001;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'high');
ac_modFilt = filtfilt(b, a, ac_mod);

% Cálculo del valor absoluto de la aceleración filtrada
ac_modFilt = abs(ac_modFilt);

% Filtro Paso Bajo a la aceleración
frecCorte = 5;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'low');
ac_modFilt = filtfilt(b, a, ac_modFilt);

% Detección de los datos por debajo del umbral
umbral = ac_modFilt < ac_umbral;

%% REPRESENTAR DATOS DEL ACELERÓMETRO Y DEL GIROSCOPIO

% Además se representa el módulo de la aceleración filtrada y los tramos de
% datos por debajo del umbral.

figure('Position', [9 39 900 600], 'Number', 'off', 'Name',...
    'Datos del Sensor');
sensor(1) = subplot(2,1,1);
    hold on;
    plot(tiempo, gx, 'r');
    plot(tiempo, gy, 'g');
    plot(tiempo, gz, 'b');
    title('Giroscopio');
    xlabel('Tiempo (s)');
    ylabel('Velocidad angular (rad/s)');
    legend('X', 'Y', 'Z');
    hold off;
sensor(2) = subplot(2,1,2);
    hold on;
    plot(tiempo, ax, 'r');
    plot(tiempo, ay, 'g');
    plot(tiempo, az, 'b');
    plot(tiempo, ac_modFilt, ':k');
    plot(tiempo, umbral, 'k', 'LineWidth', 2);
    title('Acelerómetro');
    xlabel('Tiempo (s)');
    ylabel('Aceleración (g)');
    legend('X', 'Y', 'Z', 'Acel. Filtrada', 'Umbral');
    hold off;
linkaxes(sensor,'x');

%% CALCULAR ORIENTACIÓN

quat = zeros(length(tiempo), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% Convergencia inicial
%tConvInicial = 0;
%sel = 1 : find(sign(tiempo-(tiempo(1)+tConvInicial))+1, 1);
%for i = 1:2000
%    AHRSalgorithm.UpdateIMU([0 0 0], [mean(ax(sel)) mean(ay(sel)) mean(az(sel))]);
%end

% Orientación de todos los datos
for t = 1:length(tiempo)
    if(umbral(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gx(t) gy(t) gz(t)]), [ax(t) ay(t) az(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

%% CALCULAR ACELERACIONES DE TRASLACIÓN EN EJES ABSOLUTOS

% Aplicar rotación por cuaterniones
acel = quaternRotate([ax ay az], quaternConj(quat));

% Convertir a m/s²
acel = acel * 9.81;

% Representar aceleraciones frente al tiempo
figure('Position', [9 39 900 300], 'Number', 'off', 'Name',...
    'Aceleraciones absolutas');
hold on;
plot(tiempo, acel(:,1), 'r');
plot(tiempo, acel(:,2), 'g');
plot(tiempo, acel(:,3), 'b');
title('Aceleraciones absolutas');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s²)');
legend('X', 'Y', 'Z');
hold off;

%% CALCULAR VELOCIDADES

acel(:,3) = acel(:,3) - 9.81; % Sustraer la gravedad de la aceleración en z

% Integrar la aceleración para obtener la velocidad. Iguala a cero los
% valores situados por debajo del umbral de aceleración nula.

vel = zeros(size(acel));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acel(t,:) * dt;
    if(umbral(t) == 1)
        vel(t,:) = [0 0 0];
    end
end


% Calcular el error de integración de las velocidades tras el umbral

errorVel = zeros(size(vel));
inicioUmbral = find([0; diff(umbral)] == -1);
finUmbral = find([0; diff(umbral)] == 1);

for i = 1:numel(finUmbral)
    errorEstim = vel(finUmbral(i)-1, :) /...
        (finUmbral(i) - inicioUmbral(i));
    enum = 1:(finUmbral(i) - inicioUmbral(i));
    error = [enum'*errorEstim(1) enum'*errorEstim(2) enum'*errorEstim(3)];
    errorVel(inicioUmbral(i):finUmbral(i)-1, :) = error;
end

%if length(inicioUmbral)<length(finUmbral)
%    inicioUmbral=[0; inicioUmbral];
%elseif length(inicioUmbral)>length(finUmbral)
%    finUmbral=[finUmbral; n];
%end

%for i = 1:numel(finUmbral)
%    errorEstim = vel(finUmbral(i)-1, :) /...
%        (finUmbral(i) - inicioUmbral(i));
%    enum = 1:(finUmbral(i) - inicioUmbral(i));
%    err = [enum'*errorEstim(1) enum'*errorEstim(2) enum'*errorEstim(3)];
%    if (inicioUmbral(i)-finUmbral(i))>0
%        errorVel(inicioUmbral(i):finUmbral(i)-1, :) = err;
%    end
%end

% Eliminar el error de integración
vel = vel - errorVel;

% Representar velocidades frente al tiempo
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocidades');
hold on;
plot(tiempo, vel(:,1), 'r');
plot(tiempo, vel(:,2), 'g');
plot(tiempo, vel(:,3), 'b');
title('Velocidades');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
legend('X', 'Y', 'Z');
hold off;

%% CALCULAR POSICIONES

% Integrar la velocidad para obtener la posición
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * dt;
end

% Representa la posición frente al tiempo
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Posiciones');
hold on;
plot(tiempo, pos(:,1), 'r');
plot(tiempo, pos(:,2), 'g');
plot(tiempo, pos(:,3), 'b');
title('Posiciones');
xlabel('Tiempo (s)');
ylabel('Posición (m)');
legend('X', 'Y', 'Z');
hold off;

% Aplica un filtro de paso alto para eliminar el error. Ésto hace que
% siempre que se quede quieto se desplace lentamente hacia el origen.

orden = 1;
frecCorte = 0.8;
[b, a] = butter(orden, (2*frecCorte)/(1/dt), 'high');
posHP = filtfilt(b, a, pos);

% Mantengo constantes los últimos 2 segundos de la posición
n3s=floor(n-3/dt);

for i=n3s:n
    posHP(i,:)=posHP(n3s,:);
end

% Representa la posición filtrada frente al tiempo
figure('Number', 'off', 'Name', 'Posición filtrada');
hold on;
plot(tiempo, posHP(:,1), 'r');
plot(tiempo, posHP(:,2), 'g');
plot(tiempo, posHP(:,3), 'b');
xlabel('Tiempo (s)');
ylabel('Posicion (m)');
title('Posición filtrada');
legend('X', 'Y', 'Z');

%% ANIMACIÓN 3D

%posPlot = pos;
posPlot = posHP;
quatPlot = quat;

% Retrasa el final de la animación
extraTime = 0;
onesVector = ones(extraTime*(floor(1/dt)), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*...
    onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*...
    onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Crear la animación 3D
SamplePlotFreq = 12;
disp('Animación en')
disp('3')
pause(1)
disp('2')
pause(1)
disp('1')
pause(1)
disp('Animación...')
SixDOFanimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 768],...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)',...
                'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false,...
                'AVIfps', (floor((floor(1/dt))...
                / SamplePlotFreq)));