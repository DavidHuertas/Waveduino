%% PASOS PREVIOS

clc
clear all
close all
addpath('Quaternions');	%Incluye librería de cuaterniones.

%% ABRIR EL PUERTO SERIE

puerto = 'COM3';
s = serial (puerto,'DataBits',8,'StopBits',1,'BaudRate',115200,'Parity',...
    'none');
fopen(s);
disp('ABIERTO EL PUERTO SERIE EN "s"')

%% LECTURA DE DATOS

n=50000;
t=5;

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
temp=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

disp('Calibrando, espere cinco segundos...')
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

graw=norm([mean(ax) mean(ay) mean(az)]);

temp=35-(-13200-temp)./280;

gain_ac=1/graw;

gain_gyr=1./(14.375+((15.3733-14.375)/(32-25)).*(temp-25));   

offset_gyr=[mean(gx),mean(gy),mean(gz)];

ax=ax*gain_ac;
ay=ay*gain_ac;
az=az*gain_ac;
gx=(gx-offset_gyr(1)).*gain_gyr;
gy=(gy-offset_gyr(2)).*gain_gyr;
gz=(gz-offset_gyr(3)).*gain_gyr;

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
ac_umbral = mean(ac_modFilt) + std(ac_modFilt);

%% REPRESENTAR DATOS DEL ACELERÓMETRO Y DEL GIROSCOPIO

% Además se representa el módulo de la aceleración filtrada y los tramos de
% datos por debajo del umbral.

figure('Position', [9 39 900 600], 'Number', 'off', 'Name',...
    'Calibrado del Sensor');
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
    plot(tiempo, ac_umbral, 'k', 'LineWidth', 2);
    title('Acelerómetro');
    xlabel('Tiempo (s)');
    ylabel('Aceleración (g)');
    legend('X', 'Y', 'Z', 'Acel. Filtrada', 'Umbral');
    hold off;
linkaxes(sensor,'x');

%% CALCULAR CONVERGENCIA INICIAL

quat = zeros(length(tiempo), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% Convergencia inicial
for i = 1:n
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(ax) mean(ay) mean(az)]);
end

