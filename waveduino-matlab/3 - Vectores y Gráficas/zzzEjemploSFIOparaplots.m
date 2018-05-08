%SUDDEN_EXPANSION
%ALUMNOS: MARTÍN BERENGUER CALVO    gamma=2,  a=1,  Perfil uniforme
%         GERMÁN DIEZ ESPINOSA      gamma=4,  a=0,  Perfil uniforme


clc; clear;

%discretizacion del dominio

Re=40;
L=0.4*Re;
gamma=4;
a=0;
h=1;
U=1;

N=100; M=50; itemax=1000; tol=1.0e-5; W=1.8;


x=linspace(0,L,N);
y=linspace(0,gamma,M);


dx=L/(N-1);  dy=gamma/(M-1);  Beta=dx/dy;

% hi y hs adimensionales

hi=(gamma-a-1)/2;
hs=(gamma-a+1)/2;


% Definición de algunos puntos significativos de la geometría

nhi=round((hs+h)/dy)+1;       
nhs=round((hs)/dy)+1;


%Condiciones de contorno

%F es la función de corriente
%w es la vorticidad

F=zeros(M,N);

 
F(nhi:M,1)=0;                    %Pared1
F(M,1:N)=0;                      %Pared2
F(1:nhs,1)=1;                    %Pared3
F(1,1:N)=1;                      %Pared4
F(nhs:nhi,1)=1-(y(nhs:nhi)-hs);  %Entrada
F(1:M,N)=1-(-(2/(gamma)^3)*y(1:M)'.^3+(3/(gamma)^2)*y(1:M)'.^2);    %Salida                         


%Condición inicial

w=zeros(M,N);wnew=zeros(M,N);
ite=0;

u=zeros(M,N);
v=zeros(M,N);
 
Uo=1;
Vo=1;

dt= 0.5*min((Re*dx^2)/(2*(1+Beta^2)),2/(Re*(Uo^2+Vo^2)));
ddd=1;
tol2=0.1*tol;

while ddd>tol && ite<itemax
    ite=ite+1;dd2=1; 
    w=wnew;
    while dd2>tol2
    Fold=F;
   for i=2:M-1
      for j=2:N-1
          F(i,j)=(0.5/(1+Beta^2))*(F(i,j-1)+F(i,j+1)+Beta^2*(F(i+1,j)+F(i-1,j)+dy^2*w(i,j)));
          F(i,j)=W*F(i,j)+(1-W)*Fold(i,j);
      end
   end
   
      dd2=max(max(abs(F-Fold)));

    end
    for i=2:M-1
       for j=2:N-1
   u(i,j)=(F(i-1,j)-F(i+1,j))/(2*dy);
   v(i,j)=(F(i,j-1)-F(i,j+1))/(2*dx);
       end 
    end
    
%   Velocidad en la entrada y la salida
  u(nhs:nhi,1)=1;
  u(1:M,N)=-(6/gamma^3)*y(1:M)'.^2+(6/gamma^2)*y(1:M)';
    
%   Actualizamos las Condiciones de Contorno para w en las paredes
    
  wnew(M,1:N)=3*(F(M,1:N)-F(M-1,1:N))/(dy)^2 -0.5*wnew(M-1,1:N);      %pared de abajo
  wnew(1,1:N)=3*(F(1,1:N)-F(2,1:N))/(dy)^2 -0.5*wnew(2,1:N);          %pared de arriba
  wnew(1:nhs,1)=3*(F(1:nhs,1)-F(1:nhs,2))/(dy)^2 -0.5*wnew(1:nhs,2);  %pared de entrada(3)
  wnew(nhi:M,1)=3*(F(nhi:M,1)-F(nhi:M,2))/(dy)^2 -0.5*wnew(nhi:M,2);  %pared de entrada(1)
  
%   Vorticidad en la entrada y en la salida
   
  wnew(nhs:nhi,1)=0;                              %entrada
  wnew(1:M,N)=(-12/gamma^3)*y(1:M)'+(6/gamma^2);  %salida
             
      
      for i=2:M-1
         for j=2:N-1
    
       dif=((w(i,j+1)-2*w(i,j)+w(i,j-1))/dx^2)+((w(i-1,j)-2*w(i,j)+w(i+1,j))/dy^2);
       conv=(((u(i,j+1)*w(i,j+1))-(u(i,j-1)*w(i,j-1)))/(2*dx))+(((v(i-1,j)*w(i-1,j))-(v(i+1,j)*w(i+1,j)))/(2*dy));
       wnew(i,j)=w(i,j)+dt*((dif/Re)-conv);
         end
      end
      ddd=max(max(abs(wnew-w)));
      disp([ite ddd]);
      
end


%RESULTADOS:


%Lineas de coriente

  figure('Name','Lineas de corriente'); contour(x,4-y,F,50); pause(0.1); title('Lineas de corriente'); 
  xlabel('Longitud del canal'); ylabel('Altura del canal');
  axis([0 12 0 gamma]);  
  
%Lineas de vorticidad     

  figure('Name','Lineas de vorticidad'); contour(x,4-y,wnew,200); pause(0.1); title ('Lineas de vorticidad');
  xlabel('Longitud del canal'); ylabel('Altura del canal');
  axis([0 12 0 gamma]); 
  
%Perfiles de velocidad para distintas posiciones del canal

  figure('Name','Perfiles de velocidad');
  hold on

  subplot(2,3,1), plot(u(:,1),4-y);  axis([0 1.2 0 gamma]);
  xlabel('Velocidad en la entrada'); ylabel('Altura del canal');
  subplot(2,3,2), plot(u(:,2),4-y);  axis([0 1.2 0 gamma]);
  xlabel('Velocidad en x (2)'); ylabel('Altura del canal');
  subplot(2,3,3), plot(u(:,5),4-y); axis([0 1.2 0 gamma]);
  xlabel('Velocidad en x (5)'); ylabel('Altura del canal');
  subplot(2,3,4), plot(u(:,10),4-y); axis([0 1.2 0 gamma]);
  xlabel('Velocidad en x (10)'); ylabel('Altura del canal');
  subplot(2,3,5), plot(u(:,60),4-y); axis([0 1.2 0 gamma]);
  xlabel('Velocidad en x (60)'); ylabel('Altura del canal');
  subplot(2,3,6), plot(u(:,80),4-y); axis([0 1.2 0 gamma]);
  xlabel('Velocidad en la salida'); ylabel('Altura del canal');
  
 
%Perfil del esfuerzo cortante en las paredes del canal

 Taus(1,:)=(1/Re)*(((3*u(1,:)-4*u(2,:)+u(3,:))/2*dx)+((3*v(1,:)-4*v(2,:)+v(3,:))/2*dy));         %Pared superior
 Taui(M,:)=(1/Re)*(((3*u(M,:)-4*u(M-1,:)+u(M-2,:))/2*dx)+((3*v(M,:)-4*v(M-1,:)+v(M-2,:))/2*dy)); %Pared inferior
 
 figure('Name','Esfuerzos cortantes en las paredes');
  
 subplot(2,1,1), plot(x,Taus); xlim([0 L]); grid on;
 xlabel('Longitud del canal'); ylabel('Esfuerzo cortante'); title('Pared superior');
 subplot(2,1,2), plot(x,Taui); xlim([0 L]); grid on;
 xlabel('Longitud del canal'); ylabel('Esfuerzo cortante'); title('Pared inferior');
 
 
%Valores de las longitudes de las zonas de recirculación li y ls
 
 for i=1:N
     if Taus(1,i)>0 && Taus(1,i+1)<0    %Para la pared superior
         Ls=0.5*(x(i)+x(i+1));       
     end
     
     if Taui(M,i)>0 && Taui(M,i+1)<0    %Para la pared inferior
         Li=0.5*(x(i)+x(i+1));
     end
 end



 
 