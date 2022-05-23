
%%
clear; clc;
% rng('default'); % set random seed, to get the same radom values 

%% uniformly retarded motion, system initialization, 
x0 = [0; 5]; % initial position and velocity
P0 = [0.01 0; 0 0.01]; % covariance matrix of x0, a guess
delta_t = 0.5; % motion duration
u = -2; % acceleration 

F = [1 delta_t; 0 1]; % model matrix of such motion
B = [1/2*delta_t^2; delta_t]; % model matrix of such motion

Q = 0.1 * eye(2); % covariance from external uncertainty, a guess
H = [1 0; 0 1]; % measurement matrix, measure x directly by certain sensors
R = [0.05, 0; 0, 0.08]; % covariance of the measurement parameter, determined by sensors

%% prediction after delta_t
y1 = [2.2; 4.3]; % get measurement from sensor

x1 = F*x0 + B*u; % prediction from the motion model
P1 = F*P0*F.' + Q; % covariance of x1

%% correction 
K_ = P1*H.'/(H*P1*H.' + R);  % Kalmam gain
x1_ = x1 + K_*(y1 - H*x1); % state prediction correction
P1_ = P1 - K_*H*P1; % covariance correction

%% plot results

n = 100; % the number of repetitive measurements at each time point
p0s = randn(1, n) * sqrt(P0(1,1)) + x0(1,1);
v0s = randn(1, n) * sqrt(P0(2,2)) + x0(2,1);

p1s = randn(1, n) * sqrt(P1(1,1)) + x1(1,1);
v1s = randn(1, n) * sqrt(P1(2,2)) + x1(2,1);

p1_s = randn(1, n) * sqrt(P1_(1,1)) + x1_(1,1);
v1_s = randn(1, n) * sqrt(P1_(2,2)) + x1_(2,1);

yps = randn(1, n) * sqrt(R(1,1)) + y1(1,1);
yvs = randn(1, n) * sqrt(R(2,2)) + y1(2,1);


figure(1);
scatter(p0s, v0s, 'ro'); axis equal;
hold on
scatter(p1s, v1s, 'bo'); 
scatter(yps, yvs, 'ko'); 
scatter(p1_s, v1_s, 'g*'); 
hold off
legend('x0', 'x1-model', 'x1-sensor', 'x1-kalman');
xlabel('position'); ylabel('velocity');
title('car motion prediction (one step)');


