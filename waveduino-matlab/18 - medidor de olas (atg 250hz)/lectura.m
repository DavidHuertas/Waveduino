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
t=10;

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

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
        i=i+1;
    end

end

disp('Finalizada la recogida de datos')
