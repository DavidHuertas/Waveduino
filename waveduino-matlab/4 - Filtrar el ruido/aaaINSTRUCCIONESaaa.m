%INSTRUCCIONES
%
%Vamos a limpiar todos esos picos que se ven en las gráficas del proceso
%anterior. Para ello vamos a utilizar dos métodos: uno en la representación
%de vectores y otro en la de los valores a tiempo real.
%
%PRIMER FILTRO: PROMEDIO EXPONENCIAL:
%
%En la representación de vectores veremos un promedio exponencial bastante
%sencillo que vimos en Métodos Numéricos y en SFIO, y que ahora mismo no 
%recuerdo muy bien cómo se llamaba (algo así como "Promedio Exponencial").
%
%Se trata de emplear un coeficiente de atenuación "alfa", aplicado en la
%siguiente ecuación:
%
%axfiltrada = (1-alfa)*axfiltradaanterior + alfa*axreal
%
%Por supuesto, alfa tiene un valor entre 0 y 1, cuanto mayor sea alfa,
%menor será el filtro que apliquemos, de manera que si establecemos alfa
%como 1 no habrá filtro. Y si establecemos alfa muy cercana a 0, el filtro
%será extremadamente estricto. Considero que un valor apropiado sería 0.5 ó
%0.4
%
%SEGUNDO FILTRO: PROMEDIO LINEAL (O SIMPLE):
%
%En el caso de la representación de valores en tiempo real vamos a emplear
%el promedio lineal, es decir, hacer la media del último valor con los
%anteriores valores medidos para el valor actual. Definiremos "n" como el
%número de valores tomados para hacer la media:
%
%n=3:
%
%axfiltrada=(axreal(i)+axreal(i-1)+axreal(i-2))/3
