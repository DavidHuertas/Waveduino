%INSTRUCCIONES:

% Lo primero es registrar datos. Se tiene que poder elegir el tiempo de
% toma de datos o en caso de fallo un máximo de 10000 valores. En caso de
% que no se llenen los 10000 valores deben rehacerse los vectores para
% eliminar los ceros sobrantes.
%
% Lo segundo es trabajar esos datos. Para ello se deben procesar de la
% siguiente manera:
%
% 1) Se debe calcular la orientación. Se puede hacer con cuaterniones o con
%    matriz de rotación.
%
% 2) Se calcula la aceleración absoluta, se pasa a m/s² y se resta g a la
%    dirección z de la aceleración.
%
% 3) Integra aceleración para obtener la velocidad y después se pasa por un
%    filtro de paso alto.
%
% 4) Integra la velocidad para obtener la posición y después se pasa por un
%    filtro de paso alto.
%
% Después se representa en 3D como animación.