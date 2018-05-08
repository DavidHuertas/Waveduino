%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VECTORES, APLICANDO UN FILTRO
%   PARA SUAVIZAR LOS PICOS MEDIDOS:
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
%       Este �ltimo apartado est� explicado paso a paso.


limpiar

puerto='COM5';

abrirSerial

sec=3;

[offset, gain] = calibrar (sec,s);

%C�digo de preparaci�n de la ventana (con botones)

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

%Creamos la variable en la que la aceleracion en x va a ser filtrada, y
%nuestro coeficiente de atenuaci�n alfa, que puede variar entre 0 y 1:

axfilt = 0;
alfa = 0.5;


while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (sec,s,offset,gain);
    
    %Aplicamos el filtro a la aceleraci�n en x:
    
    axfilt = (1-alfa)*axfilt + alfa*IMU(1,1);
    
    %Creamos una gr�fica dentro de la ventana (parte superior) en la que se
    %ver� el vector aceleracion en x ya filtrado:
    
    subplot(1,2,1)
    
    %Borramos todos los datos de los ejes actuales
    cla;
    
    %Representamos la aceleraci�n en X como un vector:
    line([0, axfilt], [0 0], 'Color', 'b', 'LineWidth', 3,...
        'Marker', 'o');
    
    %Ponemos l�mites a los ejes. Ya ser�a raro alcanzar 1m/s�, as� que el
    %l�mite lo pondremos como algo m�s del doble:
    
    limit = 2.5;
    axis([-limit limit -limit limit])
    axis square;
    grid on
    xlabel ('Aceleraci�n en X filtrada')
    
    %Calculamos el �ngulo del vector aceleraci�n resultante y lo imprimimos
    %en pantalla
    
    theta = atand(IMU(1,2)/IMU(1,1));
    title(['�ngulo del vector aceleraci�n: ' num2str(theta,'%.Of')]);

    
    %Creamos una gr�fica dentro de la ventana (parte inferior) en la que se
    %ver� el vector aceleracion en x sin filtrar:
    
    subplot(1,2,2)
    
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
    xlabel ('Aceleraci�n en X sin filtrar')
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end