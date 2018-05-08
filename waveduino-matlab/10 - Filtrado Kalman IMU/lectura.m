%LECTURA DEL IMU DURANTE 10 SEGUNDOS, REPRESENTACI�N NONREALTIME

sec = 3;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

disp('ABIERTO EL PUERTO SERIE')
i=1;
j=1;
tic

disp('RECIBIENDO DATOS DURANTE 10 SEGUNDOS');

while toc < 10
    while(s.bytesavailable>0)
        cadena = fgetl(s);
        if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            %disp(cadena);
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
    
end

dt=10/j;

meanmatrix(1,:)=axnum;
meanmatrix(2,:)=aynum;
meanmatrix(3,:)=aznum;
meanmatrix(4,:)=gxnum;
meanmatrix(5,:)=gynum;
meanmatrix(6,:)=gznum;
meanmatrix(7,:)=mxnum;
meanmatrix(8,:)=mynum;
meanmatrix(9,:)=mznum;

valuesmatrix(1,:) = meanmatrix(1,:)*gain(1) - offset(1,1);
valuesmatrix(2,:) = meanmatrix(2,:)*gain(1) - offset(1,2);
valuesmatrix(3,:) = meanmatrix(3,:)*gain(1) - offset(1,3);
valuesmatrix(4,:) = meanmatrix(4,:)*gain(2) - offset(2,1);
valuesmatrix(5,:) = meanmatrix(5,:)*gain(2) - offset(2,2);
valuesmatrix(6,:) = meanmatrix(6,:)*gain(2) - offset(2,3);
valuesmatrix(7,:) = meanmatrix(7,:)*gain(3) - offset(3,1);
valuesmatrix(8,:) = meanmatrix(8,:)*gain(3) - offset(3,2);
valuesmatrix(9,:) = meanmatrix(9,:)*gain(3) - offset(3,3);