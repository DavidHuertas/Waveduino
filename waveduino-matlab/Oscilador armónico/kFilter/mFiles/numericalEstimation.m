%Looks at the numerical estimation of the postion, velocity, and
%acceleration

%Prepares workspace
clear;
clc;
% close all;

%Loads Data
load('../Data/fallingWeightDisplacement.mat')
%   D
%   T
D = Disp - Disp(1);

%Estimages Velocity
dT = T(2) - T(1);
V = diff(D)/dT;
A = diff(V)/dT;



%Plots Velocity
figure(2);
plot(1000*T(1:end-1),V,'b','linewidth',3);
xlabel('Time (ms)','fontweight','bold','fontsize',13);
ylabel('Velocity (m/s)','fontweight','bold','fontsize',13);
title('Numerical Derivative Estimate of Velocity','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);
ylim([-6 1]);

%Plots Acceleration
figure(3);
plot(1000*T(1:end-2),A,'b','linewidth',3);
xlabel('Time (ms)','fontweight','bold','fontsize',13);
ylabel('Acceleration (m/s^2)','fontweight','bold','fontsize',13);
title('Numerical Derivative Estimate of Acceleration','fontweight','bold','fontsize',13);
set(gca,'fontweight','bold','fontsize',13);
xlim([0 1000*T(end)]);




