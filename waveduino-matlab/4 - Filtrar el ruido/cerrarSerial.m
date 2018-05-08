%CERRAR TODOS LOS PUERTOS SERIE
%
%Este script se encarga de cerrar y limpiar todas las comunicaciones con
%todos los puertos serie abiertos.

clc
clear all
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all
clc
disp ('EL PUERTO SERIE HA SIDO CERRADO')