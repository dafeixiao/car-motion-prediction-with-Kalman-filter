

clear; clc;
%% initialization
delta_t = 0.02;
x0 = [0; 5]; % initial position and velocity
P0 = [0.01 0; 0 0.01]; % covariance matrix of x0, a guess
u = -2; % acceleration 

F = [1 delta_t; 0 1]; % model matrix of such motion
B = [1/2*delta_t^2; delta_t]; % model matrix of such motion

Q = 0.01 * eye(2); % covariance from external uncertainty, a guess
H = [1 0; 0 1]; % measurement matrix, measure x directly by certain sensors
R = [1, 0; 0, 1]; % covariance of the measurement parameter, determined by sensors

t = 0 : delta_t : 2;
tn = length(t);
xt = zeros(2, tn-1);
xt_ = zeros(2, tn-1);
yt = zeros(2, tn-1);
gt = [x0(1,1) + x0(2,1)*t + 1/2*u*t.^2; x0(2,1)+u*t];

for i=1:tn-1
    % prediction
    x1 = F*x0 + B*u; % prediction from the motion model
    y1 = [randn(1)*sqrt(R(1,1)) + gt(1, i+1); randn(1)*sqrt(R(2,2)) + gt(2, i+1)]; % get measurement from sensor
    P1 = F*P0*F.' + Q; % covariance of x1
    % correction
    K_ = P1*H.'/(H*P1*H.' + R);  % Kalmam gain
    x1_ = x1 + K_*(y1 - H*x1); % state prediction correction
    P1_ = P1 - K_*H*P1; % covariance correction
    % for the next step
    x0 = x1_;
    P0 = P1_;
    
    % save values
    xt(:,i) = [randn(1)*sqrt(P1(1,1)) + x1(1, 1); randn(1)*sqrt(P1(2,2)) + x1(2, 1)];
    xt_(:,i) = [randn(1)*sqrt(P1_(1,1)) + x1_(1, 1); randn(1)*sqrt(P1_(2,2)) + x1_(2, 1)];
    yt(:,i) = y1;
    
end

%% plot results
figure(2);
plot(t(2:end), gt(1, 2:end), 'c', t(2:end), xt(1, :), 'r',...
    t(2:end), yt(1, :), 'k', t(2:end), xt_(1, :), 'b');
legend('ground truth', 'model', 'sensor', 'kalman');
xlabel('time'); ylabel('position');
title('car motion prediction-position');

figure(3);
plot(t(2:end), gt(2, 2:end), 'c', t(2:end), xt(2, :), 'r',...
    t(2:end), yt(2, :), 'k', t(2:end), xt_(2, :), 'b');

legend('ground truth', 'model', 'sensor', 'kalman');
xlabel('time'); ylabel('velocity');
title('car motion prediction-velocity');





