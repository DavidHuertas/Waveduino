%LECTURA DEL IMU DURANTE t SEGUNDOS, REPRESENTACIÓN NONREALTIME

sec = 3;
t = 10;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

i=1;
j=1;
tic
%axnum=zeros(1,10000);
%aynum=axnum;
%aznum=axnum;
%gxnum=axnum;
%gynum=axnum;
%gznum=axnum;
%mxnum=axnum;
%mynum=axnum;
%mznum=axnum;

disp(['RECIBIENDO DATOS DURANTE ', num2str(t), ' SEGUNDOS']);

while toc < t
    while(s.bytesavailable>0)
        cadena = fgetl(s);
        if (cadena(1) == 'a'&& cadena(numel(cadena)-1)=='f')
            disp(cadena);
            axi = findstr(cadena,'ax');
            ayi = findstr(cadena,'ay');
            azi = findstr(cadena,'az');
            gxi = findstr(cadena,'gx');
            gyi = findstr(cadena,'gy');
            gzi = findstr(cadena,'gz');
            mxi = findstr(cadena,'mx');
            myi = findstr(cadena,'my');
            mzi = findstr(cadena,'mz');
            fi = findstr(cadena,'f');
            
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
                ax(i) = cadena(axi+i+1);
            end
            axstr = str2num(ax);
            axnum(j) = axstr;
            
            for i = 1:1:len2-2
                ay(i) = cadena(ayi+i+1);
            end
            aystr = str2num(ay);
            aynum(j) = aystr;
            
            for i = 1:1:len3-2
                az(i) = cadena(azi+i+1);
            end
            azstr = str2num(az);
            aznum(j) = azstr;
            
            for i = 1:1:len4-2
                gx(i) = cadena(gxi+i+1);
            end
            gxstr = str2num(gx);
            gxnum(j) = gxstr;
            
            for i = 1:1:len5-2
                gy(i) = cadena(gyi+i+1);
            end
            gystr = str2num(gy);
            gynum(j) = gystr;
            
            for i = 1:1:len6-2
                gz(i) = cadena(gzi+i+1);
            end
            gzstr = str2num(gz);
            gznum(j) = gzstr;
            
            for i = 1:1:len7-2
                mx(i) = cadena(mxi+i+1);
            end
            mxstr = str2num(mx);
            mxnum(j) = mxstr;
            
            for i = 1:1:len8-2
                my(i) = cadena(myi+i+1);
            end
            mystr = str2num(my);
            mynum(j) = mystr;
            
            for i = 1:1:len9-2
                mz(i) = cadena(mzi+i+1);
            end
            mzstr = str2num(mz);
            mznum(j) = mzstr;
            
            
            j = j+1;
            
        else
            trash = fscanf(s);
        end
            
    end
    
end

dt=t/j;


valuesmatrix(1,:) = axnum*gain(1) - offset(1,1);
valuesmatrix(2,:) = aynum*gain(1) - offset(1,2);
valuesmatrix(3,:) = aznum*gain(1) - offset(1,3);
valuesmatrix(4,:) = gxnum*gain(2) - offset(2,1);
valuesmatrix(5,:) = gynum*gain(2) - offset(2,2);
valuesmatrix(6,:) = gznum*gain(2) - offset(2,3);
valuesmatrix(7,:) = mxnum*gain(3) - offset(3,1);
valuesmatrix(8,:) = mynum*gain(3) - offset(3,2);
valuesmatrix(9,:) = mznum*gain(3) - offset(3,3);