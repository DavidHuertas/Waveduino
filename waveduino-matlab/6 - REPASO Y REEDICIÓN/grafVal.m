%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VALORES EN EL TIEMPO
%
%1�)    Ejecutar el script "limpiar".
%
%2�)    Indicar el puerto serie (por defecto 'COM5').
%
%3�)    Ejecutar el script "abrirSerial"
%
%4�)    Realizamos un calibrado del IMU llamando a la funci�n de calibrado.
%       Para ello, primero definimos el n�mero de segundos que queremos que
%       dure cada toma de datos, introduci�ndolos en la variable "sec". s
%       es la variable que contiene la conexi�n con el puerto serie, ya
%       est� definida.
%       
%5�)    Abrimos una ventana (figure) en la que se va a
%       representar los distintos valores del IMU.
%
%6�)    Creamos un bucle que va a leer continuamente los datos del IMU, y
%       los va a representar en tiempo real

sec=0.5;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

if (~exist('h', 'var')||~ishandle(h))
    h = figure(1);
    ax = axes ('box', 'on');
end

if (~exist('button','var'))
    button = uicontrol('Style','pushbutton','String','Detener',...
        'pos',[0 0 50 25],'parent',h,'Callback','stop_call_val',...
        'UserData',1);
end

if (~exist('button2','var'))
    button2 = uicontrol('Style','pushbutton','String',...
        'Cerrar Puerto Serie','pos',[250 0 150 25],'parent',h,...
        'Callback','cerrarSerial','UserData',1);
end

%Introducimos el tama�o para el buffer empleado, e inicializamos la
%variable de la aceleracion en X:

buffer = 50;
i = 1:buffer;
axdatos = zeros(buffer,1);
aydatos = zeros(buffer,1);
azdatos = zeros(buffer,1);

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (s,offset,gain);
    
    %Actualizaci�n constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    aydatos = [aydatos(2:end); IMU(1,2)];
    azdatos = [azdatos(2:end); IMU(1,3)];
    %Representamos los datos de la aceleraci�n en x:
    
    plot(i,axdatos,'r');hold on
    plot(i,aydatos,'b');
    plot(i,azdatos,'g');
    axis([1 buffer -15 15]);
    xlabel('tiempo');
    ylabel('Aceleraciones: X rojo, Y azul, Z verde');
    hold off
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end