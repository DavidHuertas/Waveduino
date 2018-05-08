%INSTRUCCIONES
%
%Vamos a limpiar todos esos picos que se ven en las gr�ficas del proceso
%anterior. Para ello vamos a utilizar dos m�todos: uno en la representaci�n
%de vectores y otro en la de los valores a tiempo real.
%
%PRIMER FILTRO: PROMEDIO EXPONENCIAL:
%
%En la representaci�n de vectores veremos un promedio exponencial bastante
%sencillo que vimos en M�todos Num�ricos y en SFIO, y que ahora mismo no 
%recuerdo muy bien c�mo se llamaba (algo as� como "Promedio Exponencial").
%
%Se trata de emplear un coeficiente de atenuaci�n "alfa", aplicado en la
%siguiente ecuaci�n:
%
%axfiltrada = (1-alfa)*axfiltradaanterior + alfa*axreal
%
%Por supuesto, alfa tiene un valor entre 0 y 1, cuanto mayor sea alfa,
%menor ser� el filtro que apliquemos, de manera que si establecemos alfa
%como 1 no habr� filtro. Y si establecemos alfa muy cercana a 0, el filtro
%ser� extremadamente estricto. Considero que un valor apropiado ser�a 0.5 �
%0.4
%
%SEGUNDO FILTRO: PROMEDIO LINEAL (O SIMPLE):
%
%En el caso de la representaci�n de valores en tiempo real vamos a emplear
%el promedio lineal, es decir, hacer la media del �ltimo valor con los
%anteriores valores medidos para el valor actual. Definiremos "n" como el
%n�mero de valores tomados para hacer la media:
%
%n=3:
%
%axfiltrada=(axreal(i)+axreal(i-1)+axreal(i-2))/3
