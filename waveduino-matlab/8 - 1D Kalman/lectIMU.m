%LECTURA INSTANTÁNEA DE LOS DATOS DEL IMU
%
%Se trata de medir todos los datos del IMU en un instante determinado.
%A su vez, debemos calibrarlos teniendo en cuenta offset y ganancia.
%Después hacemos la media de todos los datos obtenidos.
%
%Después de esto, restará a los datos el offset y dividirá entre la
%ganancia (introducida en la función).
%
%La salida de esta función es una matriz de 3x3, en cada fila están los
%datos calibrados de cada uno de los sensores (accel, gyro, magn). En cada
%columna, el eje correspondiente (x, y, z):
%    _             _
%   |   ax  ay  az  |
%   |               |
%   |   gx  gy  gz  |
%   |               |
%   |_  mx  my  mz _|
%
%



function [valuesmatrix,dt] = lectIMU(s,offset,gain)

j=1;
tic
disp('RECIBIENDO DATOS DURANTE 10 SEGUNDOS');
axnum=zeros(1,10000);
aynum=zeros(1,10000);
aznum=zeros(1,10000);
gxnum=zeros(1,10000);
gynum=zeros(1,10000);
gznum=zeros(1,10000);
mxnum=zeros(1,10000);
mynum=zeros(1,10000);
mznum=zeros(1,10000);

while toc<10
    
while(j==1)
        
    cadena = fgetl(s);
    
    if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            
        salida = cadena;
        
        axi = findstr(salida,'ax');
        ayi = findstr(salida,'ay');
        azi = findstr(salida,'az');
        gxi = findstr(salida,'gx');
        gyi = findstr(salida,'gy');
        gzi = findstr(salida,'gz');
        mxi = findstr(salida,'mx');
        myi = findstr(salida,'my');
        mzi = findstr(salida,'mz');
        fi = findstr(salida,'f');
        
        len1 = ayi-axi;
        len2 = azi-ayi;
        len3 = gxi-azi;
        len4 = gyi-gxi;
        len5 = gzi-gyi;
        len6 = mxi-gzi;
        len7 = myi-mxi;
        len8 = mzi-myi;
        len9 = fi-mzi;
        
        for i = 1:1:len1-2
            ax(i) = salida(axi+i+1);
        end
        axstr = str2num(ax);
        axnum(j) = axstr;
        
        for i = 1:1:len2-2
            ay(i) = salida(ayi+i+1);
        end
        aystr = str2num(ay);
        aynum(j) = aystr;
        
        for i = 1:1:len3-2
            az(i) = salida(azi+i+1);
        end
        azstr = str2num(az);
        aznum(j) = azstr;
        
        for i = 1:1:len4-2
            gx(i) = salida(gxi+i+1);
        end
        gxstr = str2num(gx);
        gxnum(j) = gxstr;
        
        for i = 1:1:len5-2
            gy(i) = salida(gyi+i+1);
        end
        gystr = str2num(gy);
        gynum(j) = gystr;
        
        for i = 1:1:len6-2
            gz(i) = salida(gzi+i+1);
        end
        gzstr = str2num(gz);
        gznum(j) = gzstr;
        
        for i = 1:1:len7-2
            mx(i) = salida(mxi+i+1);
        end
        mxstr = str2num(mx);
        mxnum(j) = mxstr;
        
        for i = 1:1:len8-2
            my(i) = salida(myi+i+1);
        end
        mystr = str2num(my);
        mynum(j) = mystr;
        
        for i = 1:1:len9-2
            mz(i) = salida(mzi+i+1);
        end
        mzstr = str2num(mz);
        mznum(j) = mzstr;
        
        j=j+1;
        
    else
        trash = fscanf(s);
    end
            
end
end


dt=10/j;

valuesmatrix=zeros(3,3,j);
meanmatrix=zeros(3,3,j);

for i=1,j-1
meanmatrix(1,1,i)=axnum(i);
meanmatrix(1,2,i)=aynum(i);
meanmatrix(1,3,i)=aznum(i);
meanmatrix(2,1,i)=gxnum(i);
meanmatrix(2,2,i)=gynum(i);
meanmatrix(2,3,i)=gznum(i);
meanmatrix(3,1,i)=mxnum(i);
meanmatrix(3,2,i)=mynum(i);
meanmatrix(3,3,i)=mznum(i);
end

valuesmatrix(1,1,:) = meanmatrix(1,1,:)*gain(1) - offset(1,1);
valuesmatrix(1,2,:) = meanmatrix(1,2,:)*gain(1) - offset(1,2);
valuesmatrix(1,3,:) = meanmatrix(1,3,:)*gain(1) - offset(1,3);
valuesmatrix(2,1,:) = meanmatrix(2,1,:)*gain(2) - offset(2,1);
valuesmatrix(2,2,:) = meanmatrix(2,2,:)*gain(2) - offset(2,2);
valuesmatrix(2,3,:) = meanmatrix(2,3,:)*gain(2) - offset(2,3);
valuesmatrix(3,1,:) = meanmatrix(3,1,:)*gain(3) - offset(3,1);
valuesmatrix(3,2,:) = meanmatrix(3,2,:)*gain(3) - offset(3,2);
valuesmatrix(3,3,:) = meanmatrix(3,3,:)*gain(3) - offset(3,3);

end