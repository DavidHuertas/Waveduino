%INSTRUCCIONES:

% Lo primero es registrar datos. Se tiene que poder elegir el tiempo de
% toma de datos o en caso de fallo un m�ximo de 10000 valores. En caso de
% que no se llenen los 10000 valores deben rehacerse los vectores para
% eliminar los ceros sobrantes.
%
% Lo segundo es trabajar esos datos. Para ello se deben procesar de la
% siguiente manera:
%
% 1) Se debe calcular la orientaci�n. Se puede hacer con cuaterniones o con
%    matriz de rotaci�n.
%
% 2) Se calcula la aceleraci�n absoluta, se pasa a m/s� y se resta g a la
%    direcci�n z de la aceleraci�n.
%
% 3) Integra aceleraci�n para obtener la velocidad y despu�s se pasa por un
%    filtro de paso alto.
%
% 4) Integra la velocidad para obtener la posici�n y despu�s se pasa por un
%    filtro de paso alto.
%
% Despu�s se representa en 3D como animaci�n.