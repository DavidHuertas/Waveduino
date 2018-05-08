%ABRIR EL PUERTO SERIE
%
%Este script se encarga de comenzar la comunicación con el puerto serie 
%correspondiente. En mi casa, se conecta por defecto en el puerto COM5.

s = serial (puerto,'DataBits',8,'StopBits',1,'BaudRate',9600,'Parity','none');

fopen(s);

disp('ABIERTO EL PUERTO SERIE EN "s"')
