%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VECTORES, APLICANDO UN FILTRO
%   PARA SUAVIZAR LOS PICOS MEDIDOS:
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
%6º)    Creamos un bucle que va a leer continuamente los datos del IMU y
%       los va a representar como vectores.
%       Este último apartado está explicado paso a paso.


limpiar

puerto='COM5';

abrirSerial

sec=3;

[offset, gain] = calibrar (sec,s);

%Código de preparación de la ventana (con botones)

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
%nuestro coeficiente de atenuación alfa, que puede variar entre 0 y 1:

axfilt = 0;
alfa = 0.5;


while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANTÁNEOS del IMU:    
    IMU = lectIMU (sec,s,offset,gain);
    
    %Aplicamos el filtro a la aceleración en x:
    
    axfilt = (1-alfa)*axfilt + alfa*IMU(1,1);
    
    %Creamos una gráfica dentro de la ventana (parte superior) en la que se
    %verá el vector aceleracion en x ya filtrado:
    
    subplot(1,2,1)
    
    %Borramos todos los datos de los ejes actuales
    cla;
    
    %Representamos la aceleración en X como un vector:
    line([0, axfilt], [0 0], 'Color', 'b', 'LineWidth', 3,...
        'Marker', 'o');
    
    %Ponemos límites a los ejes. Ya sería raro alcanzar 1m/s², así que el
    %límite lo pondremos como algo más del doble:
    
    limit = 2.5;
    axis([-limit limit -limit limit])
    axis square;
    grid on
    xlabel ('Aceleración en X filtrada')
    
    %Calculamos el ángulo del vector aceleración resultante y lo imprimimos
    %en pantalla
    
    theta = atand(IMU(1,2)/IMU(1,1));
    title(['Ángulo del vector aceleración: ' num2str(theta,'%.Of')]);

    
    %Creamos una gráfica dentro de la ventana (parte inferior) en la que se
    %verá el vector aceleracion en x sin filtrar:
    
    subplot(1,2,2)
    
    %Borramos todos los datos de los ejes actuales
    cla;
    
    %Representamos la aceleración en X como un vector:
    line([0, IMU(1,1)], [0 0], 'Color', 'b', 'LineWidth', 3,...
        'Marker', 'o');
    
    %Ponemos límites a los ejes. Ya sería raro alcanzar 1m/s², así que el
    %límite lo pondremos como algo más del doble:
    
    limit = 2.5;
    axis([-limit limit -limit limit])
    axis square;
    grid on
    xlabel ('Aceleración en X sin filtrar')
    
    %Obligamos a MATLAB a representar constantemente el gráfico
    
    drawnow;
end