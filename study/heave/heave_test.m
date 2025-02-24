clear;
close all;
clc;

%%加载升沉数据
data = readtable("50s_90mm幅值_0.2hz_sin波形.csv");

% 90mm0.1hz-14-18-20
% 90mm0.2hz-14-14-18
% 90mm0.5hz-14-10-25
% line_0.1Hz_106mm
% line_0.1Hz_106mm_B
% 100s_110mm_0.2Hz_sin5
% 50s_90mm幅值_0.2hz_sin波形

%%加载数据
GRAVITY = 9.81;%重力加速度
n = height(data);%数据行数
acc_n = zeros(n,3);
data.bias_n_z = zeros(n,1);
data.velocity = zeros(n,1);
data.heave = zeros(n,1);

%%
Fs = 100;   % 采样频率
Fc = 0.06;  % 高通截止频率 (Hz),为了同时保证高低频率升沉的效果，应该动态的决定滤波频率
Fc2 = 10.5;  % 低通截止频率
dt = 1/Fs;  % 采样时间
% 归一化截止频率（相对于奈奎斯特频率）
Wn = Fc / (Fs / 2);

%% 设计二阶巴特沃斯高通 低通 滤波器 
[b_hp, a_hp] = butter(2, Wn, 'high');
[b_lp, a_lp] = butter(3, Fc2 / (Fs / 2), 'low');
t = (0:n-1) * dt; % 时间向量 (秒)

%% 设计全通滤波器
main_freq = 0.2;  % Hz, 已知的主频率
desired_phase = 10;  % 度, 期望补偿的相位

% 计算全通滤波器系数
omega = 2 * pi * main_freq / Fs;
phase_rad = desired_phase * pi / 180;
tan_half = tan(omega/2);
alpha = (tan_half - tan(phase_rad/2)) / (tan_half + tan(phase_rad/2));

%% 构建全通滤波器系数
b_ap = [alpha, 1];        % 分子系数 [alpha, 1]
a_ap = [1, alpha];        % 分母系数 [1, alpha]


log.X = zeros(n,4);
log.X2 = zeros(n,4);
log.est_freq = zeros(n,1);

%kf初始化
KF = initializeKF(dt);%KF1处理滤波前
KF2 = initializeKF(dt);%KF2处理滤波后
Qb2n = [data.quat_w, data.quat_x,data.quat_y,data.quat_z];%加载四元数
acc_b = [data.acc_x, data.acc_y, data.acc_z] * GRAVITY;%加载加速度数据

%非线性频率估计器初始化
% 参数设置,调节稳定性和响应速度
f_up = 0.5; % 最大频率 1Hz 
frq_est = aranovskiy_freq_est(Fs, f_up);

for i = 1:n
    acc_n(i,:) = qmulv(Qb2n(i,:) , acc_b(i,:));
    acc_n(i,3) = acc_n(i,3) - GRAVITY;
    KF = predictKF(KF, acc_n(i,3));
    KF = updateKF(KF, 0);
    log.X(i,:) = KF.x';%记录数据
end

y = filter(b_hp, a_hp, acc_n); % high pass
y = filter(b_ap, a_ap, y); % all pass
%y = filter(b_lp, a_lp, y); % low pass


for i = 1:n
    KF2 = predictKF(KF2, y(i,3));
    KF2 = updateKF(KF2, 0);
    log.X2(i,:) = KF2.x';

    %非线性正弦频率估计
    log.est_freq(i) = frq_est.update(log.X2(i, 3));
end

% 提取升沉和速度
heave = log.X(:, 2); % 升沉 (第 2 列)
velocity = log.X(:, 3); % 速度 (第 3 列)
% 提取升沉和速度
heave2 = log.X2(:, 2); % 升沉 (第 2 列)
velocity2 = log.X2(:, 3); % 速度 (第 3 列)

%% 改进的相位差分析
% 1. 先对信号进行预处理
signal1 = acc_n(:,3) - mean(acc_n(:,3)); % 去除直流分量
signal2 = y(:,3) - mean(y(:,3));

% 2. 使用FFT直接分析
N = length(signal1);
freq = (0:N-1)*(Fs/N);
Y1 = fft(signal1);
Y2 = fft(signal2);

% 3. 找到主要频率成分（排除0Hz和Nyquist频率）
[~, peak_indices] = findpeaks(abs(Y1(1:floor(N/2))), 'MinPeakHeight', max(abs(Y1))/10);
valid_peaks = peak_indices(freq(peak_indices) > 0.05); % 排除过低频率
main_freq_idx = valid_peaks(1); % 使用第一个有效峰值

% 4. 在主频率处计算相位差
phase1 = angle(Y1(main_freq_idx));
phase2 = angle(Y2(main_freq_idx));
phase_diff = (phase1 - phase2) * 180/pi;
% 将相位差规范化到 [-180, 180] 范围内
phase_diff = mod(phase_diff + 180, 360) - 180;

fprintf('主频率: %.3f Hz\n', freq(main_freq_idx));
fprintf('相位差: %.2f 度\n', phase_diff);

% 5. 绘制频谱图用于验证
figure('Name', 'Frequency Analysis');
subplot(2,1,1);
plot(freq(1:floor(N/2)), abs(Y1(1:floor(N/2))));
hold on;
plot(freq(1:floor(N/2)), abs(Y2(1:floor(N/2))));
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
legend('Raw', 'Filtered');
title('Frequency Spectrum');

% 在主频率处添加标记
plot(freq(main_freq_idx), abs(Y1(main_freq_idx)), 'ro', 'MarkerSize', 10);

% 6. 绘制局部时域对比
subplot(2,1,2);
t = (0:N-1)/Fs;
window_size = round(5*Fs); % 显示5秒数据
start_idx = round(N/2); % 从中间开始
plot(t(start_idx:start_idx+window_size), signal1(start_idx:start_idx+window_size));
hold on;
plot(t(start_idx:start_idx+window_size), signal2(start_idx:start_idx+window_size));
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
legend('Raw', 'Filtered');
title('Time Domain Comparison (5s window)');


%% n系加计绘图
figure('Name', 'Navigation Frame Acceleration');
plot(t, acc_n(:,3), '.-');
hold on;
plot(t, y(:,3), '.-');
legend('Raw', 'Filtered');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;

%% 升沉和速度信息绘图
figure('Name', 'Heave Motion Analysis');

% 升沉子图
subplot(2, 1, 1);
plot(t, heave, '.-');
hold on;
plot(t, heave2, '.-');
plot(t, data.heave, '.-');
grid on;
xlabel('Time (s)');
ylabel('Heave (m)');
legend('Raw', 'Filtered', 'Reference');

% 速度子图
subplot(2, 1, 2);
plot(t, velocity, '.-');
hold on;
plot(t, velocity2, '.-');
plot(t, data.velocity, '.-');
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Raw', 'Filtered', 'Reference');

%% 频率估计
figure('Name', 'Frequency Estimation');
plot(t, log.est_freq, '.-');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
grid on;



%% 函数
function r = qmulv(q, v)
    % qmulv - 使用单位四元数旋转三维向量
    %
    % 输入:
    %   q - 单位四元数 [w, x, y, z] (1x4 或 4x1)
    %   v - 三维向量 [x, y, z] (1x3 或 3x1)
    %
    % 输出:
    %   r - 旋转后的三维向量 [x, y, z] (1x3)
    %
    % 注意:
    %   假设四元数 q 已归一化。为性能考虑，不进行归一化检查。

    % 确保输入是行向量
    if size(q, 1) > 1
        q = q'; % 转置为行向量
    end
    if size(v, 1) > 1
        v = v'; % 转置为行向量
    end

    % 提取四元数分量
    q0 = q(1); % 标量部分
    q1 = q(2); % x 分量
    q2 = q(3); % y 分量
    q3 = q(4); % z 分量

    % 提取向量分量
    vx = v(1);
    vy = v(2);
    vz = v(3);

    % 预计算常用项 (减少重复计算)
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q3 = q2 * q3;

    % 计算旋转后的向量
    r(1) = (q0q0 + q1q1 - q2q2 - q3q3) * vx + ...
           2 * (q1q2 - q0q3) * vy + ...
           2 * (q1q3 + q0q2) * vz;

    r(2) = 2 * (q1q2 + q0q3) * vx + ...
           (q0q0 - q1q1 + q2q2 - q3q3) * vy + ...
           2 * (q2q3 - q0q1) * vz;

    r(3) = 2 * (q1q3 - q0q2) * vx + ...
           2 * (q2q3 + q0q1) * vy + ...
           (q0q0 - q1q1 - q2q2 + q3q3) * vz;

    % 确保输出为行向量
    r = r(:)';
end

% 初始化卡尔曼滤波器状态
function kf = initializeKF(dt)
    % 定义状态变量维度
    n = 4; % 状态维度 (位置、升沉、速度、偏差)
    %
    T = dt;
    % 定义观测变量维度
    m = 1; % 观测维度 (假设观测的是位置和升沉)
    
    % 初始化状态向量 x
    kf.x = zeros(n, 1); % 初始状态向量 [pos, heave, vel, bias]
    
    % 初始化协方差矩阵 P (初始不确定性)
    pos_std = 0.1;    % 位置积分标准差
    heave_std = 0.1;  % 升沉标准差
    vel_std = 0.01;   % 速度标准差
    bias_std = 0.03;  % acc偏差标准差
    kf.P = diag([pos_std^2, heave_std^2, vel_std^2, bias_std^2]); % 初始 P 矩阵
    
    % 初始化过程噪声协方差矩阵 Q
    pos_integral_std = 20 / 360; % 位置积分标准差
    heave_noise_std = 1.4 / 360;  % 升沉噪声标准差
    vel_noise_std = 0.1 / 360;    % 速度噪声标准差
    bias_noise_std = 0.1 / 3600;   % 偏差噪声标准差
    kf.Q = diag([pos_integral_std^2, heave_noise_std^2, vel_noise_std^2, bias_noise_std^2])*dt; % Q 矩阵
    % 初始化观测噪声协方差矩阵 R (根据实际传感器噪声设置)
    pos_meas_std = 0.64;   % 位置测量噪声标准差
    kf.R = diag(pos_meas_std^2); % R 矩阵
    
    % 初始化状态转移矩阵 F (假设简单的线性模型)
    kf.F = [1, T, 0.5*T^2, -T^3/6;
            0, 1, T,       -0.5*T^2;
            0, 0, 1,       -T;
            0, 0, 0,       1];
    % 控制输入矩阵 B
    kf.B = [T^3/6;
            0.5*T^2;
            T;
            0];
    % 初始化观测矩阵 H (假设观测位置和升沉)
    kf.H = [1, 0, 0, 0];  % 观测位置
end

function kf = predictKF(kf, u)
    % 时间更新（预测步骤）
    % kf: 卡尔曼滤波器结构体
    % u: 控制输入（例如加速度）
    
    kf.x = kf.x + kf.B * u;
    % 预测状态向量
    kf.x = kf.F * kf.x ;

    % 预测协方差矩阵
    kf.P = kf.F * kf.P * kf.F' + kf.Q;
end

function kf = updateKF(kf, z)
    % 观测更新（测量更新）
    % kf: 卡尔曼滤波器结构体
    % z: 当前观测值
    % 更新状态向量
    y = z - kf.H * kf.x;            % 观测残差
%     kf.R = 0.8 *kf.R + 0.2*(y*y); %自适应R
    % 计算卡尔曼增益
    S = kf.H * kf.P * kf.H' + kf.R; % 观测预测协方差
    K = kf.P * kf.H' / S;           % 卡尔曼增益
    kf.x = kf.x + K * y;

    % 更新协方差矩阵
    I = eye(size(kf.P));            % 单位矩阵
    kf.P = (I - K * kf.H) * kf.P;
    kf.P = 0.5 * (kf.P + kf.P');
end

