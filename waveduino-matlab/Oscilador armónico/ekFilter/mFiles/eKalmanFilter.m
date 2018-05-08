%Creates the Extended Kalman Filter

%Prepares workspace
clear;
clc;
close all;

%Loads Data
load('../Data/weightSpring.mat')
%   Disp      1x5329            42632  double
%   T         1x5329            42632  double

%Initializes Matrices
X = [24; -60; 0.1; 3];
H = [1 0 0 0];

%Initializes Errors
P = diag([1e4 1e6 100 10]);

%Sets Constants
T           = [1:length(Disp)]/120.12;
T           = T;
dT          = (T(2) - T(1));
numInter    = 50;
ddT         = dT/numInter;

%Process Noise
Oa = 100;

%Measurement Noise
R = 0.25;

%Processes data with Kalman Filter
Xs = zeros(4,length(Disp));
Ps = zeros(4,length(Disp));
Ks = zeros(4,length(Disp));
for Index = 1:min([620 length(Disp)])
    %Apriori Update
    for Interp = 1:numInter
        
        %Creates F Matrix
        F = createTransition(X);
        A = createJacobian(X);
        
        %Carries out RK4 for state propagation
        k1      = F*ddT;
        F2      = createTransition(X + 0.5*k1);
        k2      = F2*ddT;
        F3      = createTransition(X + 0.5*k2);
        k3      = F3*ddT;
        F4      = createTransition(X + k3);
        k4      = F4*ddT;
        Delta   = 1/6*(k1 + 2*k2 + 2*k3 + k4);
        X       = X + Delta;
        
        %Integrates Covariance
        Fund    = expm(A*ddT);
        Qt      = createQ(X,Oa,ddT);
        P       = Fund*P*Fund' + Qt;
        
    end
    
    %Measurement Update
    K = P*H'*(H*P*H' + R)^-1;
    E = Disp(Index) - H*X;
    X = X + K*E;
    P = (eye(4) - K*H)*P;
    
    %Storage
    Xs(:,Index)     = X;
    Ps(:,Index)     = diag(P);
    Ks(:,Index)     = K;
    
end

%Plots Results
figure(1);
plot(T,Disp,'b','linewidth',3);
hold on;
plot(T,Xs(1,:),'r','linewidth',3);
hold off;
xlim([0 5]);
xlabel('Time (S)','fontweight','bold','fontsize',13);
ylabel('Position (cm)','fontweight','bold','fontsize',13);
title('Position State Variable and Measured Position','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);

pM = filtfilt(ones(1,5)/5,1,Xs(3,:));
figure(2);
plot(T,Xs(3,:),'linewidth',3);
hold on;
plot(T,pM+sqrt(Ps(3,:)),'--k','linewidth',1.5);
plot(T,pM-sqrt(Ps(3,:)),'--k','linewidth',1.5);
hold off;
ylim([-0.1 0.8]);
xlim([0 5]);
xlabel('Time (S)','fontweight','bold','fontsize',13);
ylabel('Damping Ratio','fontweight','bold','fontsize',13);
title('Damping State Variable','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);

pM = filtfilt(ones(1,5)/5,1,Xs(4,:));
figure(4);
plot(T,Xs(4,:),'linewidth',3);
hold on;
plot(T,pM+sqrt(Ps(4,:)),'--k','linewidth',1.5);
plot(T,pM-sqrt(Ps(4,:)),'--k','linewidth',1.5);
hold off;
ylim([0 10]);
xlim([0 5]);
xlabel('Time (S)','fontweight','bold','fontsize',13);
ylabel('Natural Frequency (Rad/S)','fontweight','bold','fontsize',13);
title('Natural Frequency State Variable','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);

figure(5);
plot(abs(Ks'));
legend('Position','Velocity','Damping','Frequency');


%Final State Value
% X =
%
%     0.2465
%    33.8937
%     0.0698
%     4.3744





