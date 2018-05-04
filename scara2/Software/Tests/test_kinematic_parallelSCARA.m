clear all; clc

%% Input for test

l1 = 0.25;
l2 = 0.25;

% phi0 = [-pi/3, pi/2  , pi/4  , -pi/2, -pi/2]; %, 0   , -pi/2, -pi/4, pi/4]
% phi1 = [ -pi/6, 3*pi/4, 3*pi/4, -pi/4,  0   ]; %, pi/2, 0    ,  pi/4, pi/2]
% 
% x = [0.25, 0.25, 0   , -0.25, 0    ];
% y = [0.25, 0   , 0.25,  0   , -0.25];
% 
% %% Direct kinematic
% 
% for i = 1:length(phi0)
%     x_dirKin(i) = l1 * cos(phi0(i)) + l2 * cos(phi1(i));
%     y_dirKin(i) = l1 * sin(phi0(i)) + l2 * sin(phi1(i));
% end
% 
% % x = x_dirKin
% % y = y_dirKin

%% Inverse kinematic

x = [-0.23, -0.12, -0.06]; % -0.23, -0.19, -0.1  ];
y = [-0.25, -0.25, -0.25]; %-0.40, -0.28, -0.001];
% y = [0.40, 0.40, 0.22];

for i = 1:length(x)
    alpha = acos((l1*l1 + l2*l2 - x(i)*x(i) - y(i)*y(i))/(2*l1*l2));
    delta_q = pi - alpha;
    beta = atan2(y(i),x(i))
%     gamma = asin((l2*sin(alpha))/(sqrt(x(i)*x(i)+y(i)*y(i))));
    gamma = acos((x(i)*x(i) + y(i)*y(i) + l1*l1 - l2*l2)/(2*l1*sqrt(x(i)*x(i) + y(i)*y(i))));
    
    phi0_invKin(i) = beta - gamma;
    phi1_invKin(i) = phi0_invKin(i) + delta_q;

end

% phi0
% phi1
% x_dirKin
% y_dirKin

% x
% y
phi0_invKin
phi1_invKin
