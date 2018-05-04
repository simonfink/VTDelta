clear all
close all
clc

% Zone
x = -50*pi/180:0.01:250*pi/180;

amax = 480; % max Beschleunigung [rad/s^2]
vmax = 24.44; % max geschwindigkeit [rad/s]

BrakingDistance = (vmax^2)/(2*amax);% [rad]

% Mot0
upperLim0 = 195*pi/180; % [rad]
underLim0 = -45*pi/180; % [rad]
upperDangerzone0 = (upperLim0 - BrakingDistance);%*180/pi % [°]
underDangerzone0 = (underLim0 + BrakingDistance);%*180/pi % [°]

M0underVeloSaturation = sqrt(2*amax*(x-underLim0)); 
M0upperVeloSaturation = -sqrt(2*-amax*(x-upperLim0)); 


% Mot1
upperLim1 = 225*pi/180; % [rad]
underLim1 = -15*pi/180; % [rad]
upperDangerzone1 = (upperLim1 - BrakingDistance);%*180/pi % [°]
underDangerzone1 = (BrakingDistance + underLim1);%*180/pi % [°]

M1underVeloSaturation = sqrt(2*amax*(x-underLim1)); 
M1upperVeloSaturation = -sqrt(2*-amax*(x-upperLim1)); 

%% Plots

% Mot0
h1 = figure;
plot(x,M0underVeloSaturation,'b','LineWidth',2)
grid on
title('Geschwindigkeits Begrenzung Mot0')
xlabel('s [rad]')
ylabel('v [rad/s]')
axis([-50*pi/180 250*pi/180 -24.44 24.44])
hold on
plot(x,M0upperVeloSaturation,'g','LineWidth',2)
legend( 'M0underVeloSaturation','M0upperVeloSaturation')
set(h1,'color',[1 1 1])

%Mot1
h2 = figure;
plot(x,M1underVeloSaturation,'b','LineWidth',2)
grid on
title('Geschwindigkeits Begrenzung Mot1')
xlabel('s [rad]')
ylabel('v [rad/s]')
axis([-50*pi/180 250*pi/180 -24.44 24.44])
hold on
plot(x,M1upperVeloSaturation,'g','LineWidth',2)
legend( 'M1underVeloSaturation','M1upperVeloSaturation')
set(h2,'color',[1 1 1])