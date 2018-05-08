%% Pasos previos

clc
clear all
close all
addpath('quaternion_library');	% include quaternion library

% Abrir el puerto serie

puerto = 'COM3';
s = serial (puerto,'DataBits',8,'StopBits',1,'BaudRate',9600,'Parity','none');
fopen(s);
disp('ABIERTO EL PUERTO SERIE EN "s"')
 
%% Lectura de datos

n=10000;
t=60;

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);
mx=zeros(n,1);
my=zeros(n,1);
mz=zeros(n,1);

disp('Recibiendo datos')
tic

while (i<=n && toc<=t)
    
    imu=fgetl(s);
    if (imu(1)=='k')
        ax(i)=str2double(fgetl(s));
        ay(i)=str2double(fgetl(s));
        az(i)=str2double(fgetl(s));
        gx(i)=str2double(fgetl(s));
        gy(i)=str2double(fgetl(s));
        gz(i)=str2double(fgetl(s));
        my(i)=str2double(fgetl(s));
        mz(i)=str2double(fgetl(s));
        mx(i)=str2double(fgetl(s));
        i=i+1;
    end

end

disp('Finalizada la recogida de datos')

if i==n
    t=toc;
    ax=(-1).*ax;
    gx=(-1).*gx;
else
    ax=(-1).*ax(1:i-1);
    ay=ay(1:i-1);
    az=az(1:i-1);
    gx=(-1).*gx(1:i-1);
    gy=gy(1:i-1);
    gz=gz(1:i-1);
    mx=mx(1:i-1);
    my=my(1:i-1);
    mz=mz(1:i-1);
    n=i-1;
end

%CERRAR TODOS LOS PUERTOS SERIE
%
%Este script se encarga de cerrar y limpiar todas las comunicaciones con
%todos los puertos serie abiertos.

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all
disp ('EL PUERTO SERIE HA SIDO CERRADO')

dt=t/n;

graw=norm([mean(ax(1:10)) mean(ay(1:10)) mean(az(1:10))]);

gain(1)=1/graw;
gain(2)=1.09/14.375;
gain(3)=1/norm([mean(mx(1:10)) mean(my(1:10)) mean(mz(1:10))]);

%offsetaz=offsetaz+norm([offsetax,offsetay,offsetaz]); 
%esto es una tontería ya que las aceleraciones no son absolutas

ax=ax*gain(1);
ay=ay*gain(1);
az=az*gain(1);
gx=(gx-mean(gx(1:10)))*gain(2);
gy=(gy-mean(gy(1:10)))*gain(2);
gz=(gz-mean(gz(1:10)))*gain(2);
mx=(mx-mean(mx(1:10)))*gain(3);
my=(my-mean(my(1:10)))*gain(3);
mz=(mz-mean(mz(1:10)))*gain(3);

matr=[ax ay az gx gy gz mx my mz]';

samplePeriod = dt;

gyr = [gx...
       gy...
       gz];        % gyroscope
acc = [ax...
       ay...
       az];        % accelerometer
mag = [mx...
       my...
       mz];        % magnetometer
  
% Plot
figure('Number', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr(:,1), 'r');
plot(gyr(:,2), 'g');
plot(gyr(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('Number', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr)
    ahrs.Update(gyr(i,:) * (pi/180), acc(i,:),mag(i,:));	% gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc));  % accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

% Plot
figure('Number', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s

% Plot
figure('Number', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');

%% High-pass filter linear velocity to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)

linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('Number', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');

%% High-pass filter linear position to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('Number', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% Play animation

SamplePlotFreq = 2;

SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script