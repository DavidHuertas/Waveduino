%Creates the Kalman Filter

%Prepares workspace
clear;
clc;
% close all;

%Loads Data
load('../Data/fallingWeightDisplacement.mat')
%   D
%   T
D = Disp - Disp(1);

%Initializes Matrices
X = [0; 0; 2.0];
H = [1 0 0];

%Initializes Errors
dP = 1;
dV = 5;
dA = 10;
P = diag([dP^2 dV^2 dA^2]);

%Sets Constants
dT  = T(2) - T(1);

%Measurement Noise
sigma = (1.0/100)^2;
R = sigma;

%Creates transition Matrix
A = expm([      0                   1           0;
                0                   0           -1;
                0                   0           0]*dT);

%Process Noise
aS = 0.01^2;
Q = [aS*dT^3/3  aS*dT^2/2   0; 
    aS*dT^2/2   aS^2*dT     0; 
    0           0           0];
            
%Processes data with Kalman Filter
Xs = zeros(3,length(D));
Ps = zeros(3,length(D));
Ks = zeros(3,length(D));

for Index = 1:length(D)

    %A-priori Updates
    X = A*X;
    P = A*P*A' + Q;
    
    %A-posteri Update
    K = P*H'*(H*P*H' + R)^-1;
    P = (eye(3) - K*H)*P;
    E = D(Index) - H*X;
    X = X + K*E;
    
    %Storage
    Xs(:,Index)     = X;
    Ps(:,Index)     = diag(P);
    Ks(:,Index)     = K;
   
end

%Plots Displacement
figure(1);
plot(1000*T,100*D,'g','linewidth',3);
hold on;
plot(1000*T,100*Xs(1,:),'b','linewidth',3);
plot(1000*T,100*(Xs(1,:)+sqrt(Ps(1,:))),'--k','linewidth',2);
plot(1000*T,100*(Xs(1,:)-sqrt(Ps(1,:))),'--k','linewidth',2);
hold off;
xlabel('Time (ms)','fontweight','bold','fontsize',13);
ylabel('Position (cm)','fontweight','bold','fontsize',13);
title('Position State Variable','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);

%Plots Velocity
figure(2);
plot(1000*T,Xs(2,:),'linewidth',3);
hold on;
plot(1000*T,(Xs(2,:)+sqrt(Ps(2,:))),'--k','linewidth',1.5);
plot(1000*T,(Xs(2,:)-sqrt(Ps(2,:))),'--k','linewidth',1.5);
hold off;
xlabel('Time (ms)','fontweight','bold','fontsize',13);
ylabel('Velocity (m/s)','fontweight','bold','fontsize',13);
title('Velocity State Variable','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);
ylim([-6 1]);

%Plots Acceleration
figure(3);
plot(1000*T,Xs(3,:),'linewidth',3);
hold on;
plot(1000*T,(Xs(3,:)+sqrt(Ps(3,:))),'--k','linewidth',1.5);
plot(1000*T,(Xs(3,:)-sqrt(Ps(3,:))),'--k','linewidth',1.5);
hold off;
xlabel('Time (ms)','fontweight','bold','fontsize',13);
ylabel('Acceleration (m/s^2)','fontweight','bold','fontsize',13);
title('Acceleration State Variable','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);

%Plots Kalman Gains
figure(4);
plot(1000*T,abs(Ks'),'linewidth',3);
legend('Position','Velocity','Acceleration');
title('Kalman Gains');
xlabel('Time (ms)','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);


