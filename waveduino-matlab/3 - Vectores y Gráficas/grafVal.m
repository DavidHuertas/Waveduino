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

puerto='COM5';

abrirSerial

sec=1;

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

buffer = 100;
i = 1:buffer;
axdatos = zeros(buffer,1);

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (s,offset,gain);
    
    %Actualizaci�n constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    
   
    %Representamos los datos de la aceleraci�n en x:
    
    plot(i,axdatos,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleraci�n en X');
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end