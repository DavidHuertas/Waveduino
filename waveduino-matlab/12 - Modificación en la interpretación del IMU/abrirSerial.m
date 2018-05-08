%ABRIR EL PUERTO SERIE
%
%Este script se encarga de comenzar la comunicación con el puerto serie 
%correspondiente. En este caso, se establece por defecto el puerto COM5.

puerto = 'COM3';

s = serial (puerto,'DataBits',8,'StopBits',1,'BaudRate',74400,'Parity','none');

fopen(s);

disp('ABIERTO EL PUERTO SERIE EN "s"')
