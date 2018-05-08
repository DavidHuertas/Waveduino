%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VALORES EN EL TIEMPO, APLICANDO
%   UN FILTRO PARA SUAVIZAR LOS PICOS MEDIDOS
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
%       los va a representar en tiempo real.
%       �ste �ltimo apartado est� explicado paso a paso.


limpiar

puerto='COM5';

abrirSerial

sec=3;

[offset, gain] = calibrar (sec,s);

if (~exist('h', 'var')||~ishandle(h))
    h = figure(1);
    ax = axes ('box', 'on');
end

if (~exist('button','var'))
    button = uicontrol('Style','pushbutton','String','Stop',...
        'pos',[0 0 50 25],'parent',h,'Callback','stop_call_val',...
        'UserData',1);
end

if (~exist('button2','var'))
    button2 = uicontrol('Style','pushbutton','String','Close Serial Port',...
        'pos',[250 0 150 25],'parent',h,'Callback','closeSerial',...
        'UserData',1);
end

%Introducimos el tama�o para el buffer empleado, e inicializamos la
%variable para la aceleraci�n en X, tanto la real como la filtrada:

buffer = 100;
i = 1:buffer;
axdatos = zeros(buffer,1);
axdatosfilt = zeros(buffer,1);

%Definimos el n�mero de datos a tomar para el filtro:

n=3;

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (sec,s,offset,gain);
    
    %Actualizaci�n constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    
    %Filtramos la aceleraci�n en X:
    axdatosfilt = [axdatosfilt(2:end);... 
        mean(axdatos(buffer:-1:buffer-n+1))];
       
    %Representamos los datos de la aceleraci�n en x ya filtrada en el
    %gr�fico superior, y sin filtrar en el inferior.
    
    subplot(2,1,1)
    plot(i,axdatosfilt,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleraci�n en X filtrada');
    
    subplot(2,1,2)
    plot(i,axdatos,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleraci�n en X sin filtrar');
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end