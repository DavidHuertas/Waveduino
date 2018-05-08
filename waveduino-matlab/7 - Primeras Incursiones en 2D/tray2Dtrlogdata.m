% TRAYECTORIA 2D SOBRE PLANO HORIZONTAL: SÓLO TRASLACIÓN.
%
% SIN FILTRADO.

sec=2;
At=0.05;
t=10;
j=t/At;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

X = zeros (2, 1000);
V = X;
A = V;
OM = zeros (1000);
TH = OM;

for i=2:j
    IMU = lectIMU(s,offset,gain);

    A(1,i)=IMU(1,1);
    A(2,i)=IMU(1,2);
    OM(i)=IMU(2,3);
    
    TH(i)=TH(i-1)+OM(i)*At; 

    A(1,i)=A(1,i)*cos(TH(i));
    A(2,i)=A(2,i)*sin(TH(i));
    
    V(:,i)=V(:,i-1)+ A(:,i)*At;

    X(:,i)=X(:,i-1)+V(:,i-1)*At+0.5*A(:,i)*(At)^2;
    
%    figure (1);
%    subplot(2,1,1);
%    plot(X(1,i),X(2,i),'r','Marker', 'o');hold on
%    axis([-1 1 -1 1]);
%    xlabel('X (m)');
%    ylabel('Y (m)');
%    hold off
    
%    subplot(2,1,2);
%    plot(A(1,i),A(2,i),'g','Marker', 'o');hold on
%    axis([-15 15 -15 15]);
%    xlabel('Ax (m)');
%    ylabel('Ay (m)');
%    hold off
    
%    drawnow;

end