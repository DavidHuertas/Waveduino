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

    offset=0;
    gain=1;
    
    %Leemos los datos del IMU en cada uno de los tres ejes.
    
    %(x,y,z)=(0,0,1)
    mbox=msgbox ('Mantenga el IMU con el eje Z hacia arriba'); uiwait(mbox);
    zmat = lectCal(sec,s,offset,gain);
    
    %(x,y,z)=(1,0,0)
    mbox=msgbox ('Mantenga el IMU con el eje X hacia arriba'); uiwait(mbox);
    xmat = lectCal(sec,s,offset,gain);
    
    %(x,y,z)=(0,1,0)
    mbox=msgbox ('Mantenga el IMU con el eje Y hacia arriba'); uiwait(mbox);
    ymat = lectCal(sec,s,offset,gain);
    
    %Calculamos offsets de cada sensor y eje. Observar bien el mapeado de
    %las tres matrices xmat, ymat, zmat.
    
    offsetaccelX = (zmat(1,1) + ymat(1,1)) / 2;
    offsetaccelY = (xmat(1,2) + zmat(1,2)) / 2;
    offsetaccelZ = (xmat(1,3) + ymat(1,3)) / 2;
    
    offsetgyroX = (zmat(2,1) + ymat(2,1)) / 2;
    offsetgyroY = (xmat(2,2) + zmat(2,2)) / 2;
    offsetgyroZ = (xmat(2,3) + ymat(2,3)) / 2;
    
    offsetmagnX = (zmat(3,1) + ymat(3,1)) / 2;
    offsetmagnY = (xmat(3,2) + zmat(3,2)) / 2;
    offsetmagnZ = (xmat(3,3) + ymat(3,3)) / 2;
    
%    offsetaccelX = (ax_z + ax_y) / 2;
 %   offsetaccelY = (ay_x + ay_z) / 2;
  %  offsetaccelZ = (az_x + az_y) / 2;
   % 
%    offsetgyroX = (gx_z + gx_y) / 2;
 %   offsetgyroY = (gy_x + gy_z) / 2;
  %  offsetgyroZ = (gz_x + gz_y) / 2;
   % 
%    offsetmagnX = (mx_z + mx_y) / 2;
 %   offsetmagnY = (my_x + my_z) / 2;
  %  offsetmagnZ = (mz_x + mz_y) / 2;
   % 
    %Calculamos ganancias (factores de escala)
    
    gainaccelX = xmat(1,1) - offsetaccelX;
    gainaccelY = ymat(1,2) - offsetaccelY;
    gainaccelZ = zmat(1,3)- offsetaccelZ;
    
    gaingyroX = xmat(2,1) - offsetgyroX;
    gaingyroY = ymat(2,2) - offsetgyroY;
    gaingyroZ = zmat(2,3) - offsetgyroZ;
    
    gainmagnX = xmat(3,1) - offsetmagnX;
    gainmagnY = ymat(3,2) - offsetmagnY;
    gainmagnZ = zmat(3,3) - offsetmagnZ;

%    gainaccelX = ax_x - offsetaccelX;
 %   gainaccelY = ay_y - offsetaccelY;
  %  gainaccelZ = az_z - offsetaccelZ;
   % 
%    gaingyroX = gx_x - offsetgyroX;
 %   gaingyroY = gy_y - offsetgyroY;
  %  gaingyroZ = gz_z - offsetgyroZ;
   % 
%    gainmagnX = mx_x - offsetmagnX;
 %   gainmagnY = my_y - offsetmagnY;
  %  gainmagnZ = mz_z - offsetmagnZ;
   % 
    
    offset = [offsetaccelX,offsetaccelY,offsetaccelZ,offsetgyroX,...
        offsetgyroY,offsetgyroZ,offsetmagnX,offsetmagnY,offsetmagnZ];
    
    gain = [gainaccelX,gainaccelY,gainaccelZ,gaingyroX,gaingyroY,...
        gaingyroZ,gainmagnX,gainmagnY,gainmagnZ];
    
    mbox = msgbox ('Calibrado del sensor COMPLETADO');
end