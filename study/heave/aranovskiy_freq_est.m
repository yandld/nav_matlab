%% Aranovskiy Frequency Estimator Implementation
% 参考论文：The New Algorithm of Sinusoidal Signal Frequency Estimation
% 作者：Alexey A. Bobtsov, et al.

clear;clc;close all;

%% 仿真参数设置
fs = 100;              % 采样频率 Hz
dt = 1/fs;            % 采样时间间隔
t = 0:dt:20;          % 仿真时间向量

% 真实信号参数
f_true = 0.2;         % 真实频率 Hz
omega_true = 2*pi*f_true;
A = 1;                % 信号幅度

% 生成测试信号
y = A * sin(omega_true * t);

% 可选：添加噪声
noise = 0.1 * randn(size(t));
y = y + noise;

%% 估计器参数设置
f_up = 1;            % 假设的频率上限 Hz
omega_up = 2*pi*f_up;
params.a = omega_up;  % 滤波器参数
params.b = params.a;  % 滤波器参数
params.k = 2;        % 增益参数
params.omega_up = omega_up;

%% 初始化状态变量
state.x1_hat = 0;
state.theta = -(params.omega_up^2/4);  % 基于频率上限的初始猜测
state.sigma = state.theta;

%% 运行估计器
N = length(t);
f_est = zeros(1, N);
x1_hist = zeros(1, N);
theta_hist = zeros(1, N);

for i = 1:N
    [f_est(i), x1_hist(i), theta_hist(i), state] = aranovskiy_freq_estimate(y(i), dt, params, state);
end

%% 绘图
figure('Name', 'Aranovskiy Frequency Estimator Results');

% 频率估计结果
subplot(3,1,1);
plot(t, f_est, 'r-', t, f_true*ones(size(t)), 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Frequency (Hz)');
legend('Estimated', 'True');
title('Frequency Estimation');

% 状态观测器输出
subplot(3,1,2);
plot(t, x1_hist, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x1 state');
title('State Observer Output');

% 输入信号
subplot(3,1,3);
plot(t, y, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Input Signal');

function [f_est, x1_hat, theta, state] = aranovskiy_freq_estimate(y, dt, params, state)
    % Aranovskiy频率估计器实现
    % 输入:
    %   y: 当前测量值
    %   dt: 采样时间间隔
    %   params: 参数结构体，包含 a, b, k, omega_up
    %   state: 状态结构体，包含 x1_hat, theta, sigma
    % 输出:
    %   f_est: 估计频率 (Hz)
    %   x1_hat: 状态观测器输出
    %   theta: θ参数
    %   state: 更新后的状态结构体
    
    % 提取参数
    a = params.a;
    b = params.b;
    k = params.k;
    
    % 状态观测器更新 (公式6)
    x1_dot = -a * state.x1_hat + b * y;
    x1_hat = state.x1_hat + dt * x1_dot;
    
    % σ动态更新 (公式10)
    sigma_dot = -k * state.x1_hat^2 * state.theta ...
                - k * a * state.x1_hat * x1_dot ...
                - k * b * x1_dot * y;
    sigma = state.sigma + dt * sigma_dot;
    
    % θ更新 (公式11)
    theta = sigma + k * b * x1_hat * y;
    
    % 频率估计
    omega_est = sqrt(abs(theta));  % 使用abs防止负值
    f_est = omega_est / (2*pi);
    
    % 更新状态结构体
    state.x1_hat = x1_hat;
    state.theta = theta;
    state.sigma = sigma;
end
