%LECTURA DEL IMU DURANTE 10 SEGUNDOS, REPRESENTACIÓN NONREALTIME

disp('ABIERTO EL PUERTO SERIE')
pause(3);
i=1;
j=1;
tic

disp('RECIBIENDO DATOS DURANTE 10 SEGUNDOS');

while toc < 10
    while(s.bytesavailable>0)
        cadena = fgetl(s);
        if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            disp(cadena);
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


figure(1);plot3(axnum,aynum,aznum);grid on;xlabel('X');ylabel('Y');zlabel('Z');title('ACELERACIÓN');
figure(2);plot(1:j-1,axnum);title('ACELERÓMETRO, X');
figure(3);plot(1:j-1,aynum);title('ACELERÓMETRO, Y');
figure(4);plot(1:j-1,aznum);title('ACELERÓMETRO, Z');
figure(5);plot(1:j-1,gxnum);title('GIROSCOPIO, X');
figure(6);plot(1:j-1,gynum);title('GIROSCOPIO, Y');
figure(7);plot(1:j-1,gznum);title('GIROSCOPIO, Z');
figure(8);plot(1:j-1,mxnum);title('MAGNETOSCOPIO, X');
figure(9);plot(1:j-1,mynum);title('MAGNETOSCOPIO, Y');
figure(10);plot(1:j-1,mznum);title('MAGNETOSCOPIO, Z');
