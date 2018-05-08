%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VALORES EN EL TIEMPO
%
%1º)    Ejecutar el script "limpiar".
%
%2º)    Indicar el puerto serie (por defecto 'COM5').
%
%3º)    Ejecutar el script "abrirSerial"
%
%4º)    Realizamos un calibrado del IMU llamando a la función de calibrado.
%       Para ello, primero definimos el número de segundos que queremos que
%       dure cada toma de datos, introduciéndolos en la variable "sec". s
%       es la variable que contiene la conexión con el puerto serie, ya
%       está definida.
%       
%5º)    Abrimos una ventana (figure) en la que se va a
%       representar los distintos valores del IMU.
%
%6º)    Creamos un bucle que va a leer continuamente los datos del IMU, y
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

%Introducimos el tamaño para el buffer empleado, e inicializamos la
%variable de la aceleracion en X:

buffer = 100;
i = 1:buffer;
axdatos = zeros(buffer,1);

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANTÁNEOS del IMU:    
    IMU = lectIMU (s,offset,gain);
    
    %Actualización constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    
   
    %Representamos los datos de la aceleración en x:
    
    plot(i,axdatos,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleración en X');
    
    %Obligamos a MATLAB a representar constantemente el gráfico
    
    drawnow;
end