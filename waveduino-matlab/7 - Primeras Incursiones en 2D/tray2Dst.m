% TRAYECTORIA 2D SOBRE PLANO HORIZONTAL: SÓLO TRASLACIÓN.
%
% SIN FILTRADO.

sec=2;
At=0.01;
t=10;
j=t/At;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

X = zeros (2, 10000);
V = X;
A = V;

for i=2:j
    IMU = lectIMU(s,offset,gain);

    A(1,i)=IMU(1,1);
    A(2,i)=IMU(1,2);

    V(:,i)=V(:,i-1)+ A(:,i)*At;

    X(:,i)=X(:,i-1)+V(:,i-1)*At;
    
    figure (1);
    subplot(2,1,1);
    plot(X(1,i),X(2,i),'r','Marker', 'o');hold on
    axis([-1 1 -1 1]);
    xlabel('X (m)');
    ylabel('Y (m)');
    hold off
    
    subplot(2,1,2);
    plot(A(1,i),A(2,i),'g','Marker', 'o');hold on
    axis([-15 15 -15 15]);
    xlabel('Ax (m)');
    ylabel('Ay (m)');
    hold off
    
    drawnow;
    
    pause (0.01)
end