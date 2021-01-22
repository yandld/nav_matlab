
close all;  
clear;
clc;  


%% load data
m=load('data/data.mat');

t = double(m.whatever_data.t);

x_init  = m.whatever_data.x_init; % initial x position [m]
y_init  = m.whatever_data.y_init; % initial y position [m]
th_init = m.whatever_data.th_init; % initial theta position [rad]


% input signal
v = m.whatever_data.v;  % translational velocity input [m/s]
om = m.whatever_data.om;  % rotational velocity input [rad/s]

% bearing and range measurements, LIDAR constants
b = m.whatever_data.b;  % bearing to each landmarks center in the frame attached to the laser [rad]
r = m.whatever_data.r;  % range measurements [m]
l = double(m.whatever_data.l); % x,y positions of landmarks [m]
d = double(m.whatever_data.d);%distance between robot center and laser rangefinder [m]


%% init filter

v_var = 0.01;  % translation velocity variance  
om_var = 0.01;  % rotational velocity variance 
r_var = 0.001;  % range measurements variance
b_var = 0.001; % bearing measurement variance

Q_km = diag([v_var, om_var]); % input noise covariance 
cov_y = diag([r_var, b_var]);  % measurement noise covariance 

x_est = zeros([length(v), 3]);  % estimated states, x, y, and theta
P_est = zeros([length(v), 3, 3]);  % state covariance matrices

x_est(1,:) = [x_init, x_init, th_init]; % initial state
P_est = diag([1, 1, 0.1]); % initial state covariance


%% main filter loop
 x_check = x_est(1,:);
  P_check = P_est;
for k = 2:length(t)

    
    delta_t = t(k) - t(k-1); % time step (difference between timestamps)
    x_check(3) = wrapToPi( x_check(3));
    theta = x_check(3);
    % 1. Update state with odometry readings (remember to wrap the angles to [-pi,pi])
    x_check(1) = x_check(1) + v(k-1) * cos(theta) * delta_t;
    x_check(2) =  x_check(2)  + v(k-1) * sin(theta) * delta_t;
    x_check(3) =  x_check(3) +  om(k-1) * delta_t;
    x_check(3) = wrapToPi(x_check(3));
    
    
    % 2. Motion model jacobian with respect to last state
    F_km = [1, 0, -v(k-1)*sin(theta)*delta_t; 0, 1, v(k-1)*cos(theta)*delta_t; 0 0 1];
    % 3. Motion model jacobian with respect to noise
    L_km = [cos(theta)*delta_t, 0; sin(theta)*delta_t, 0; 0 delta_t];

% 4. Propagate uncertainty
    P_check = F_km * P_check * F_km' + L_km * Q_km * L_km';
    
%5. Update state estimate using available landmark measurements
for i = 1:size(r,2)
      [ x_check, P_check] = measurement_update(l(i,:), r(k, i), b(k, i), d, cov_y, P_check, x_check);
end

% Set final state predictions for timestep
    x_est(k,:) = x_check;
    %P_est = P_check;
end

%% show
subplot(2,1,1);
plot(x_est(:,1), x_est(:,2), '.');
title('Estimated trajectory');
xlabel('x [m]');
ylabel('x [m])');

subplot(2,1,2);
plot(t, x_est(:,3));
title('Estimated trajectory');
xlabel('Time [s]');
ylabel('theta [rad]');


%%
function [x, k] = measurement_update(lk, rk, bk, d, cov_y, P_check, x_check)
    x_check(3) = wrapToPi(x_check(3));
    x = x_check;
    P = P_check;
    x_k = x(1);
    y_k = x(2);
    theta_k = x(3);
    x_l = lk(1);
    y_l = lk(2);
    
    dx = x_l - x_k - d * cos(theta_k);
    dy = y_l - y_k - d * sin(theta_k);
     r = sqrt(dx^(2) + dy^(2));
     phi = atan2(dy, dx) - theta_k;
     y = [r, wrapToPi(phi)]';
     y_meas = [rk, wrapToPi(bk)]';
    
    % 1. Compute measurement Jacobian
    M = eye(2);
    H = ones(2, 3);
    H(1, 1) = -dx / r;
    H(1, 2) = -dy / r;
    H(1, 3) = d * (dx * sin(theta_k) - dy * cos(theta_k)) / r;
    H(2, 1) = dy / r^(2);
    H(2, 2) = -dx / r^(2);
    H(2, 3) = -d * (dy * sin(theta_k) + dx * cos(theta_k)) / r^(2);
    
    % 2. Compute Kalman Gain
    K = P * H' * inv(H * P * H' + M * cov_y * M');
    
    % 3. Correct predicted state (remember to wrap the angles to [-pi,pi])
    x_check = x + (K * (y_meas - y))';
    x_check(3) = wrapToPi(x_check(3));
    
    % 4. Correct covariance
     P_check = (eye(3) - K * H) * P;
     
    x = x_check;
    k = P_check;
end


