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



function valuesmatrix = lectIMU(s,offset,gain)

valuesmatrix=zeros(3);
meanmatrix=zeros(3);
i=1;
j=1;

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
        axnum = axstr;
        
        for i = 1:1:len2-2
            ay(i) = salida(ayi+i+1);
        end
        aystr = str2num(ay);
        aynum = aystr;
        
        for i = 1:1:len3-2
            az(i) = salida(azi+i+1);
        end
        azstr = str2num(az);
        aznum = azstr;
        
        for i = 1:1:len4-2
            gx(i) = salida(gxi+i+1);
        end
        gxstr = str2num(gx);
        gxnum = gxstr;
        
        for i = 1:1:len5-2
            gy(i) = salida(gyi+i+1);
        end
        gystr = str2num(gy);
        gynum = gystr;
        
        for i = 1:1:len6-2
            gz(i) = salida(gzi+i+1);
        end
        gzstr = str2num(gz);
        gznum = gzstr;
        
        for i = 1:1:len7-2
            mx(i) = salida(mxi+i+1);
        end
        mxstr = str2num(mx);
        mxnum = mxstr;
        
        for i = 1:1:len8-2
            my(i) = salida(myi+i+1);
        end
        mystr = str2num(my);
        mynum = mystr;
        
        for i = 1:1:len9-2
            mz(i) = salida(mzi+i+1);
        end
        mzstr = str2num(mz);
        mznum = mzstr;
        
        j=j+1;
        
    else
        trash = fscanf(s);
    end
            
end

meanmatrix(1,1)=axnum;
meanmatrix(1,2)=aynum;
meanmatrix(1,3)=aznum;
meanmatrix(2,1)=gxnum;
meanmatrix(2,2)=gynum;
meanmatrix(2,3)=gznum;
meanmatrix(3,1)=mxnum;
meanmatrix(3,2)=mynum;
meanmatrix(3,3)=mznum;

valuesmatrix(1,:) = meanmatrix(1,:)*gain(1) - offset(1,:);
valuesmatrix(2,:) = meanmatrix(2,:)*gain(2) - offset(2,:);
valuesmatrix(3,:) = meanmatrix(3,:)*gain(3) - offset(3,:);

end