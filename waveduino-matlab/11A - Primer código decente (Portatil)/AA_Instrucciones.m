% INSTRUCCIONES PARA CALCULAR Y REPRESENTAR EL GIRO ALREDEDOR DE Z
%
% -Cerramos todos los puertos serie por si hubiera alguno abierto
%
% -Limpiamos ventana de comandos, variables y demás
%
% -Abrimos el puerto serie en cuestión (Arduino)
%
% -Configuramos el archivo "lectura":
%
%   -"sec" es el número de segundos para el calibrado
%   -"t" es el número de segundos en los que se toman los datos
%   -El script devuelte una matriz de 9xn, donde cada fila almacena los
%   datos del IMU en orden de aceleraciones, giros y magnet, en x, y, z. A
%   su vez, devuelve también el paso de tiempo como "dt"
%
% -Una vez leidos los datos, el archivo "kalman" debe filtrar los datos del
% giro
%
% -Por último, el archivo "giro" debe integrar el giro y representar en un
% gráfico el ángulo girado en función del tiempo.