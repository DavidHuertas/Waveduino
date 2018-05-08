clear all
clc
s = serial ('COM5','DataBits',8,'StopBits',1,'BaudRate',9600,'Parity','none');

fopen(s);

disp('ABIERTO EL PUERTO SERIE')
pause(3);
i=1;
j=1;
tic
while toc < 10
    while(s.bytesavailable>0)
        cadena = fgetl(s);
        if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            disp('RECIBIENDO DATOS DURANTE 10 SEGUNDOS');
            
            salida = cadena;
            
            axi = findstr(salida,'ax');
            ayi = findstr(salida,'ay');
            azi = findstr(salida,'az');
            gxi = findstr(salida,'gx');
            
            len1 = ayi-axi;
            len2 = azi-ayi;
            len3 = gxi-azi;
            
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
            

            
            figure(1);plot3(axnum,aynum,aznum);grid on;xlabel('X');ylabel('Y');zlabel('Z');title('ACELERACIÓN');
            
            j = j+1;
            
        else
            trash = fscanf(s);
        end
            
    end
    
end
fclose(s);
delete(s);
clear s;