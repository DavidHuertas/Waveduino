%LECTURA INICIAL DE LOS DATOS DEL IMU 
%
%Se trata de medir todos los datos del IMU durante unos segundos. Después 
%hacemos la media de todos los datos obtenidos. El número de segundos para
%la toma de medidas es sec.
%
%La salida de esta función es una matriz de 3x3, en cada fila están las
%medias de las aceleraciones, momentos, y mediciones del magnetoscopio. En
%cada columna, las mediciones en x, y, z.

function meanmatrix = lecturamedias(sec,s)

meanmatrix=zeros(3);
i=1;
j=1;
pause(2)
tic

while toc < sec
    while(s.bytesavailable>0)
        cadena = fgetl(s);
        if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            disp('RECIBIENDO DATOS DURANTE UNOS SEGUNDOS, POR FAVOR ESPERE');
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
            
            
            j = j+1;
            
        else
            trash = fscanf(s);
        end
            
    end

meanmatrix(1,1)=mean(axnum);
meanmatrix(1,2)=mean(aynum);
meanmatrix(1,3)=mean(aznum);
meanmatrix(2,1)=mean(gxnum);
meanmatrix(2,2)=mean(gynum);
meanmatrix(2,3)=mean(gznum);
meanmatrix(3,1)=mean(mxnum);
meanmatrix(3,2)=mean(mynum);
meanmatrix(3,3)=mean(mznum);

end