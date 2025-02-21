clear;
close all;
%%加载升沉数据
data = readtable("50s_90mm幅值_0.2hz_sin波形.csv");
%%加载数据
GRAVITY = 9.81;%重力加速度
n = height(data);%数据行数
acc_n = zeros(n,3);
data.bias_n_z = zeros(n,1);
data.velocity = zeros(n,1);
data.heave = zeros(n,1);

%%
Fs = 100;   %采样频率
Fc = 0.10;  % 高通截止频率 (Hz),为了同时保证高低频率升沉的效果，应该动态的决定滤波频率
Fc2 = 2.5;  %低通截止频率
dt = 1/Fs;  %采样时间
% 归一化截止频率（相对于奈奎斯特频率）
Wn = Fc / (Fs / 2);

% 设计二阶巴特沃斯高通滤波器
[b, a] = butter(5, Wn, 'high');
[b2, a2] = butter(3, Fc2 / (Fs / 2), 'low');
t = (0:n-1) * dt; % 时间向量 (秒)

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
esta = 1.0;  % 低通滤波器系数
estb = 1.0;  % 观测器增益
estk = 1.0;  % 估计器增益

% 初始化非线性频率估计器
Est = initialize_nl_sin_frq_est(esta, estb, estk);

for i = 1:n
    acc_n(i,:) = qmulv(Qb2n(i,:) , acc_b(i,:));
    acc_n(i,3) = acc_n(i,3) - GRAVITY;
    KF = predictKF(KF, acc_n(i,3));
    KF = updateKF(KF, 0);
    log.X(i,:) = KF.x';%记录数据
end

y = filter(b, a, acc_n-mean(acc_n)); % 滤波后的信号
y = filter(b2, a2, y); % 滤波后的信号
for i = 1:n
    KF2 = predictKF(KF2, y(i,3));
    KF2 = updateKF(KF2, 0);
    log.X2(i,:) = KF2.x';
    %非线性正弦频率估计
    Est = nl_sin_frq_est_update(Est, y(i,3), dt);
    log.est_freq(i) = Est.freq;%记录频率
end
% 提取升沉和速度
heave = log.X(:, 2); % 升沉 (第 2 列)
velocity = log.X(:, 3); % 速度 (第 3 列)
% 提取升沉和速度
heave2 = log.X2(:, 2); % 升沉 (第 2 列)
velocity2 = log.X2(:, 3); % 速度 (第 3 列)
%% n系加计绘图
figure;
plot(t,acc_n(:,3));hold on;
plot(t,y(:,3));
legend("acc_n_z","acc_n_z_filter")
title("acc_n");

%% 升沉和速度信息绘图
figure;

% 绘制升沉
subplot(2, 1, 1); % 创建第一个子图
plot(t, heave, 'b-', 'LineWidth', 1.5); % 滤波前，蓝色实线
hold on;
plot(t, heave2, 'r--', 'LineWidth', 1.5); % 滤波后，红色虚线
plot(t, data.heave, 'g:', 'LineWidth', 1.5); % HI数据，绿色点划线
grid on;
title('Heave (升沉)');
xlabel('Time (s)');
ylabel('Heave');
legend("滤波前","滤波后","HI");

% 绘制速度
subplot(2, 1, 2); % 创建第二个子图
plot(t, velocity, 'b-', 'LineWidth', 1.5); % 速度曲线，蓝色
hold on;
plot(t, velocity2, 'r-', 'LineWidth', 1.5); % 速度曲线，红色
plot(t, data.velocity, 'g-', 'LineWidth', 1.5); % 速度曲线，绿色
grid on;
title('Velocity (速度)');
xlabel('Time (s)');
ylabel('Velocity');
legend("滤波前","滤波后");
% 调整图像布局
sgtitle('Heave and Velocity'); % 总标题

%% 绘制非线性器频率估计
figure;
plot(log.est_freq(1:end));

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
    vel_noise_std = 0.1 / 3600;    % 速度噪声标准差
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

% 四元数转欧拉角函数（Z-X-Y 旋转顺序）
function euler = quat2euler_ZXY(qnb)
    q11 = qnb(1)*qnb(1); q12 = qnb(1)*qnb(2); q13 = qnb(1)*qnb(3); q14 = qnb(1)*qnb(4);
    q22 = qnb(2)*qnb(2); q23 = qnb(2)*qnb(3); q24 = qnb(2)*qnb(4); 
    q33 = qnb(3)*qnb(3); q34 = qnb(3)*qnb(4);
    q44 = qnb(4)*qnb(4);
    C12=2*(q23-q14);
    C22=q11-q22+q33-q44;
    C31=2*(q24-q13); C32=2*(q34+q12); C33=q11-q22-q33+q44;
    
    pitch = asind(C32);
    roll = atan2d(-C31,C33);
    yaw = -atan2d(C12,C22);
    %yaw = yaw + (yaw<0)*360;
    euler = [roll; pitch; yaw];
end
%非线性频率估计器初始化
function est = initialize_nl_sin_frq_est(a, b, k)
    % Initialize the nl_sin_frq_est_t structure with given parameters
    % Parameters:
    %   a - Low-pass filter coefficient
    %   b - Observer gain
    %   k - Estimator gain
    % Returns:
    %   est - Initialized structure

    % Parameters
    est.a = a;      % Low-pass filter coefficient
    est.b = b;      % Observer gain
    est.k = k;      % Estimator gain

    % States (initialized to zero)
    est.y = 0.0;    % Measurement signal
    est.x1 = 0.0;   % State estimate
    est.theta = 0.0; % Frequency parameter estimate
    est.sigma = 0.0; % Auxiliary variable
    est.omega = 0.0; % Estimated angular frequency
    est.freq = 0.0;  % Estimated frequency in Hz
end
%非线性正弦函数估计器
function est = nl_sin_frq_est_update(est, y, dt)
    % Update frequency estimate with new measurement
    % Implementation based on:
    % "The New Algorithm of Sinusoidal Signal Frequency Estimation"
    % by Bobtsov et al., IFAC 2013

    % Store measurement
    est.y = y;

    % Calculate state derivative
    x1_dot = -est.a * est.x1 + est.b * y;

    % Calculate auxiliary variable derivative
    sigma_dot = -est.k * est.x1^2 * est.theta ...
                - est.k * est.a * est.x1 * x1_dot ...
                - est.k * est.b * x1_dot * y;

    % Update states using Euler integration
    est.x1 = est.x1 + x1_dot * dt;
    est.sigma = est.sigma + sigma_dot * dt;

    % Update frequency estimate
    est.theta = est.sigma + est.k * est.b * est.x1 * y;
    est.omega = sqrt(abs(est.theta));
    est.freq = est.omega / (2.0 * pi);
end
