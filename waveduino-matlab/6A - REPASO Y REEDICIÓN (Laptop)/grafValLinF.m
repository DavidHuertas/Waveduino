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


sec=0.5;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

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
        'pos',[250 0 150 25],'parent',h,'Callback','cerrarSerial',...
        'UserData',1);
end

%Introducimos el tama�o para el buffer empleado, e inicializamos la
%variable para la aceleraci�n en X, tanto la real como la filtrada:

buffer = 50;
i = 1:buffer;
axdatos = zeros(buffer,1);
aydatos = zeros(buffer,1);
azdatos = zeros(buffer,1);
axdatosfilt = zeros(buffer,1);
aydatosfilt = zeros(buffer,1);
azdatosfilt = zeros(buffer,1);

%Definimos el n�mero de datos a tomar para el filtro:

n=3;

while (get(button, 'UserData'))
    
    %Obtenemos los valores INSTANT�NEOS del IMU:    
    IMU = lectIMU (s,offset,gain);
    
    %Actualizaci�n constante de los datos en pantalla:
    axdatos = [axdatos(2:end); IMU(1,1)];
    aydatos = [aydatos(2:end); IMU(1,2)];
    azdatos = [azdatos(2:end); IMU(1,3)];
    
    %Filtramos las aceleraciones:
    
    axdatosfilt = [axdatosfilt(2:end);... 
        mean(axdatos(buffer:-1:buffer-n+1))];
    aydatosfilt = [aydatosfilt(2:end);... 
        mean(aydatos(buffer:-1:buffer-n+1))];   
    azdatosfilt = [azdatosfilt(2:end);... 
        mean(azdatos(buffer:-1:buffer-n+1))];
    
    %Representamos los datos de la aceleraci�n en x ya filtrada en el
    %gr�fico superior, y sin filtrar en el inferior.
    
    subplot(2,1,1)
    plot(i,axdatosfilt,'r');hold on
    plot(i,aydatosfilt,'b');
    plot(i,azdatosfilt,'g');
    axis([1 buffer -15 15]);
    xlabel('tiempo');
    ylabel('Acel. Filtrada');
    hold off
    
    subplot(2,1,2)
    plot(i,axdatos,'r');hold on
    plot(i,aydatos,'b');
    plot(i,azdatos,'g');
    axis([1 buffer -15 15]);
    xlabel('tiempo');
    ylabel('Acel. Real');
    hold off
    
    %Obligamos a MATLAB a representar constantemente el gr�fico
    
    drawnow;
end