clear;
clc
close all;

%% EKF 2D 定位
% 参考: https://pythonrobotics.readthedocs.io/en/latest/modules/localization.html#histogram-filter-localization

%过程噪声
Q = diag([  % predict state covariance
    0.1  % variance of location on x-axis
    0.1  % variance of location on y-axis
    deg2rad(1.0)  % variance of yaw angle
    1.0 % variance of velocity
    ])^(2);

%量测噪声
R = diag([
    3 % GPS X量测噪声
    3 % GPS Y量测噪声
    ])^(2);

%  输入噪声参数(速度传感器(编码器) 和 角速度传感器(陀螺仪))
global  INPUT_NOISE
INPUT_NOISE  = diag([1.0, deg2rad(30.0)]) ^(2);

%GPS 噪声参数
global GPS_NOISE;
GPS_NOISE = diag([0.7, 0.7]) ^(2);

dt = 0.1; %积分间隔
N = 400;

%% 初始值
x_est = zeros(4, 1); %卡尔曼估计值
x_gt = zeros(4, 1); %真实值
P = eye(4); %过程噪声矩阵
x_dr = zeros(4,1); %航迹推算值

for i = 1:N
    
    u = calc_input(); % 生成理想传感器数据
    [x_gt, z, x_dr, ud] = observation(x_gt, x_dr, u, dt); %生成真实轨迹，DR轨迹 和 带有噪声的观测值
    [x_est , P] = ekf_estimation(x_est, P, Q, R, z, ud, dt); %EKF滤波
    
    % 记录历史数据
    hx_gt(i,:) = x_gt;
    hz(i,:) = z;
    hx_dr(i,:) = x_dr;
    hx_est(i,:) = x_est;
    hPtrace(i,:) = trace(P);
end

%% plot结果
figure;
hold on;
plot(hx_gt(:,1), hx_gt(:,2));
plot(hx_dr(:,1), hx_dr(:,2));
plot(hz(:,1), hz(:,2),'.');
plot(hx_est(:,1), hx_est(:,2));
legend("真实值(GT)", "航迹推算(DR)", "观测值(GPS)", "EKF滤波");

figure;
hold on;
plot(1:N, hx_gt(:,3));
plot(1:N, hx_dr(:,3));
plot(1:N, hx_est(:,3));
title("航向角");
legend("真实值(GT)", "航迹推算(DR)" , "EKF滤波");

figure;
hold on;
plot(1:N, hx_gt(:,4));
plot(1:N, hx_dr(:,4),'g');
plot(1:N, hx_est(:,4),'b.');
title("速度");
legend("真实值(GT)", "航迹推算(DR)" , "EKF滤波");


figure;
plot(1:N, hPtrace);
title("P迹(系统稳定度)");


%计算误差
pos_diff = hx_est(:,1:2) - hx_gt(:,1:2);
err = sum( sum(abs(pos_diff).^2,2).^(1/2));
fprintf("滤波位置与真实值误差:%f m\n", err);


%% 子函数

% EKF滤波过程
function  [x_est , P]  = ekf_estimation(x_est, P, Q, R, z, u, dt)

% predict
x_est = motion_model(x_est, u, dt);
JF = jacob_f(x_est, u, dt);
P = JF*P*JF' + Q;

% update
JH = jacob_h();
z_pred = observation_model(x_est);
y = z - z_pred; %计算新息
S = JH * P * JH' + R;
K = P * JH' * S^(-1);
x_est = x_est + K*y;
P = (eye(length(x_est)) - K*JH) * P;
end


%观测模型
function z =  observation_model(x)
H = [1, 0, 0, 0;  0, 1, 0, 0];
z = H * x;
end

%观测过程Jacob
function J= jacob_h()
% Jacobian of Observation Model
J= [1 0 0 0; 0 1 0 0];

end

%求状态过程Jacob
function J = jacob_f(x, u, dt)
yaw = x(3);
v = x(4);
J = [1 0 -dt*v*sin(yaw), dt*cos(yaw); 0 1 dt*v*cos(yaw) dt*sin(yaw); 0 0 1 0; 0 0 0 1];

end

%生成 真实值，DR值和观测值
function  [x_gt, z, x_dr, u_dr] = observation(x_gt, x_dr, u, dt)


x_gt =  motion_model(x_gt, u, dt);

% 添加测量噪声
global GPS_NOISE;
z = observation_model(x_gt) + GPS_NOISE* randn(2,1);

% 添加传感器噪声
global INPUT_NOISE;
u_dr = u + INPUT_NOISE * randn(2,1);

% 生成真实DR轨迹
x_dr = motion_model(x_dr, u_dr, dt);
end

% 运动模型
function x =  motion_model(x, u, dt)
F =  [
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 0];

yaw = x(3);
B = [dt*cos(yaw) 0; dt*sin(yaw) 0; 0 dt; 1 0];
x = F*x + B*u;
end

%生成传感器输入
function u =  calc_input()

v = 1.0;  % [m/s]
yawrate = 0.1; % [rad/s]

u = [v yawrate]';
end

