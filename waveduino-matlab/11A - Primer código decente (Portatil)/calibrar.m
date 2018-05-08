%CALIBRADO DEL IMU
%
%Se introduce la cantidad de tiempo que tomamos datos del IMU, seguida de
%la variable que contiene la información del puerto serie (por defecto "s")
%
%Recordamos que las matrices de salida de la función lectCal tienen la
%forma:
%    _             _
%   |   ax  ay  az  |
%   |               |
%   |   gx  gy  gz  |
%   |               |
%   |_  mx  my  mz _|
%
%

function [offset, gain] = calibrar (sec,s)

    %Valores de las ganancias de cada sensor:
    gainaccel=1/(8*3.3);
    gaingyro=3/14.375;
    gainmagn=1/1132.5;
    
    %Leemos los datos del IMU en cada uno de los tres ejes.
    
    %(x,y,z)=(0,0,-1)
    mbox=msgbox ('Mantenga el IMU con el eje Z hacia arriba'); uiwait(mbox);
    valmatrx = lectCal(sec,s);
    
    g=norm(valmatrx(1,:))*gainaccel;
    
    offsetaccelX = valmatrx(1,1)*gainaccel;
    offsetaccelY = valmatrx(1,2)*gainaccel;
    offsetaccelZ = valmatrx(1,3)*gainaccel+g;
    
    offsetgyroX = valmatrx(2,1)*gaingyro;
    offsetgyroY = valmatrx(2,2)*gaingyro;
    offsetgyroZ = valmatrx(2,3)*gaingyro;
    
    offsetmagnX = valmatrx(3,1)*gainmagn;
    offsetmagnY = valmatrx(3,2)*gainmagn;
    offsetmagnZ = valmatrx(3,3)*gainmagn;
    
    %Calculamos ganancias (factores de escala)
    
    offset = [offsetaccelX,offsetaccelY,offsetaccelZ; offsetgyroX,...
        offsetgyroY,offsetgyroZ;offsetmagnX,offsetmagnY,offsetmagnZ];
    
    gain = [gainaccel,gaingyro,gainmagn];
    
end