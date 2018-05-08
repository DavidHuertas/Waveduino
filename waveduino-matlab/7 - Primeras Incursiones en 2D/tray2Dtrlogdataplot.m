% TRAYECTORIA 2D SOBRE PLANO HORIZONTAL: SÓLO TRASLACIÓN.
%
% SIN FILTRADO.

sec=2;
At=0.01;
t=10;
j=t/At;

    figure (1);
    subplot(2,1,1);
    plot(X(1,:),X(2,:),'r','Marker', 'o');hold on
    axis([-1 1 -1 1]);
    xlabel('X (m)');
    ylabel('Y (m)');
    hold off
   
    subplot(2,1,2);
    plot(A(1,:),A(2,:),'g','Marker', 'o');hold on
    axis([-15 15 -15 15]);
    xlabel('Ax (m)');
    ylabel('Ay (m)');
    hold off
  
   

