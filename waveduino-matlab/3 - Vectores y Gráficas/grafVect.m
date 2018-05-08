%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VECTORES
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
%6�)    Creamos un bucle que va a leer continuamente los datos del IMU y
%       los va a representar como vectores.


puerto='COM5';

abrirSerial

sec=1;

[offset, gain] = calibrar (sec,s);

if (~exist('h', 'var')||~ishandle(h))
    h = figure(1);
    ax = axes ('box', 'on');
end

if (~exist('button','var'))
    button = uicontrol('Style','pushbutton','String','Stop',...
        'pos',[0 0 50 25],'parent',h,'Callback','stop_call_vect',...
        'UserData',1);
end

if (~exist('button2','var'))
    button2 = uicontrol('Style','pushbutton','String','Close Serial Port',...
        'pos',[250 0 150 25],'parent',h,'Callback','closeSerial',...
        'UserData',1);
end

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (s,offset,gain);
    
    %Borramos todos los datos de los ejes actuales
    cla;
    
    %Representamos la aceleraci�n en X como un vector:
    line([0, IMU(1,1)], [0 0], 'Color', 'b', 'LineWidth', 3,...
        'Marker', 'o');
    
    %Ponemos l�mites a los ejes. Ya ser�a raro alcanzar 1m/s�, as� que el
    %l�mite lo pondremos como algo m�s del doble:
    
    limit = 2.5;
    axis([-limit limit -limit limit])
    axis square;
    grid on
    xlabel ('Aceleraci�n en X')
    
    %Calculamos el �ngulo del vector aceleraci�n resultante y lo imprimimos
    %en pantalla
    
    theta = atand(IMU(1,2)/IMU(1,1));
    title(['�ngulo del vector aceleraci�n: ' num2str(theta,'%.Of')]);
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end