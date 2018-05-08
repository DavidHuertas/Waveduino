%
%   REPRESENTAR ACELERACIONES DEL IMU COMO VALORES EN EL TIEMPO, APLICANDO
%   UN FILTRO PARA SUAVIZAR LOS PICOS MEDIDOS
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
%       los va a representar en tiempo real.
%       Éste último apartado está explicado paso a paso.


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

%Introducimos el tamaño para el buffer empleado, e inicializamos la
%variable para la aceleración en X, tanto la real como la filtrada:

buffer = 100;
i = 1:buffer;
axdatos = zeros(buffer,1);
axdatosfilt = zeros(buffer,1);

%Definimos el número de datos a tomar para el filtro:

n=3;

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANTÁNEOS del IMU:    
    IMU = lectIMU (sec,s,offset,gain);
    
    %Actualización constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    
    %Filtramos la aceleración en X:
    axdatosfilt = [axdatosfilt(2:end);... 
        mean(axdatos(buffer:-1:buffer-n+1))];
       
    %Representamos los datos de la aceleración en x ya filtrada en el
    %gráfico superior, y sin filtrar en el inferior.
    
    subplot(2,1,1)
    plot(i,axdatosfilt,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleración en X filtrada');
    
    subplot(2,1,2)
    plot(i,axdatos,'r');
    axis([1 buffer -3.5 3.5]);
    xlabel('tiempo');
    ylabel('Valor de la Aceleración en X sin filtrar');
    
    %Obligamos a MATLAB a representar constantemente el gráfico
    
    drawnow;
end