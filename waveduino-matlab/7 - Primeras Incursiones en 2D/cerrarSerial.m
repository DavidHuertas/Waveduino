%CERRAR TODOS LOS PUERTOS SERIE
%
%Este script se encarga de cerrar y limpiar todas las comunicaciones con
%todos los puertos serie abiertos.

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
close all
disp ('EL PUERTO SERIE HA SIDO CERRADO')