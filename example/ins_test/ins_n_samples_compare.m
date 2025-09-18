% 测试代码  
clear;
data = readtable("ins_update_data.csv");
%% 
close all;
omega_sequence = [data.gyr_x data.gyr_y data.gyr_z]'/180 * 3.1415926 * 1;%%可以乘以系数模仿大机动，当然不排除是因为噪声问题
%% 处理一下数据长度
len = length(omega_sequence);
if mod(len, 4) == 0
    omega_sequence = omega_sequence(:,1:end-1);
else 
    omega_sequence = omega_sequence(:,1:end);
end
omega_sequence = omega_sequence - mean(omega_sequence(:,1:500),2);
% 初始化
T = 1 / 100;  % 系统采样周期
R2D = 180/3.1415926;
D2R = 3.1415926/180;

% 不同时间轴定义
t_single = 0:T*4:length(omega_sequence)*T;  % 陀螺仪算法时间轴 (25Hz)
t_single = t_single(1:floor(length(omega_sequence)/4)+1);  % 确保长度匹配

% 获取加速度数据并进行4Hz低通滤波
acc_data_raw = [data.acc_x data.acc_y data.acc_z];
acc_data_raw = acc_data_raw(1:4:length(omega_sequence),:);

% 加速度时间轴 (与陀螺仪算法结果对应)
t_acc = (0:size(acc_data_raw,1)-1) * (T*4);  % 25Hz采样

% 设计4Hz低通滤波器 (无相位延迟)
fs = 25;  % 采样频率 (100Hz/4 = 25Hz)
fc = 2.2;   % 截止频率 4Hz
[b, a] = butter(4, fc/(fs/2), 'low');  % 4阶巴特沃斯低通滤波器

% 对加速度数据进行零相位滤波
acc_data = zeros(size(acc_data_raw));
for i = 1:3
    acc_data(:,i) = filtfilt(b, a, acc_data_raw(:,i));
end

fprintf('加速度数据4Hz低通滤波完成\n');

% 初始姿态四元数
q0 = [data.quat_w(1); data.quat_x(1); data.quat_y(1); data.quat_z(1)]';

% 存储结果
q_single = zeros(4, length(t_single));
q_two = zeros(4, length(t_single));
q_four = zeros(4, length(t_single));
q_parc = zeros(4, length(t_single));  % PARC结果
q_single(:,1) = q0;
q_two(:,1) = q0;
q_four(:,1) = q0;
q_parc(:,1) = q0;
quat = [data.quat_w data.quat_x data.quat_y data.quat_z];
% 存储欧拉角结果 (roll, pitch, yaw)
euler_single = zeros(3, length(t_single));
euler_two = zeros(3, length(t_single));
euler_parc = zeros(3, length(t_single));
euler_four = zeros(3, length(t_single));
quat_eular = zeros(3, length(t_single));

% 加速度转横滚俯仰角结果存储
euler_acc = zeros(3, length(t_acc));  % 使用加速度自己的时间轴

% 计算初始欧拉角
euler_single(:,1) = quat2euler_ZXY(q0);
euler_two(:,1) = quat2euler_ZXY(q0);
euler_parc(:,1) = quat2euler_ZXY(q0);
euler_four(:,1) = quat2euler_ZXY(q0);

quat =quat(1:4:length(omega_sequence),:)';

% 计算所有时刻的加速度姿态角
for k = 1:length(t_acc)
    if k <= length(data.yaw)
        current_yaw = data.yaw(1+4*(k-1)) * D2R;  % 使用对应的yaw角
    else
        current_yaw = data.yaw(end) * D2R;  % 如果超出范围，使用最后一个值
    end
    euler_acc(:,k) = acc2attitude_ENU(acc_data(k,:), current_yaw);
end

% 主循环
j = 1;  % 单子样索引
for i = 1:4:length(omega_sequence)-4
    % 获取两个连续采样点的角速度
    omega1 = omega_sequence(:,i);
    omega2 = omega_sequence(:,i+1);
    omega3 = omega_sequence(:,i+2);
    omega4 = omega_sequence(:,i+3);
    %s四子样更新
    q_four(:,j+1) = AttitudeUpdate.fourSample(...
        q_four(:,j), omega1, omega2, omega3, omega4, T);
    % 双子样更新
    q_two(:,j+1) = AttitudeUpdate.twoSample(...
        q_two(:,j), omega1, omega2, T);
    q_two(:,j+1) = AttitudeUpdate.twoSample(...
        q_two(:,j+1), omega3, omega4, T);
    % 单子样更新
    q_single(:,j+1) = AttitudeUpdate.singleSample(...
        q_single(:,j), omega1, T);
    q_single(:,j+1) = AttitudeUpdate.singleSample(...
        q_single(:,j+1), omega2, T);
    q_single(:,j+1) = AttitudeUpdate.singleSample(...
        q_single(:,j+1), omega3, T);
    q_single(:,j+1) = AttitudeUpdate.singleSample(...
        q_single(:,j+1), omega4, T);
    % PARC更新
    if j > 1
        q_parc(:,j+1) = AttitudeUpdate.singleSamplePARC(...
            q_parc(:,j), omega1, omega_sequence(:,i-1), T);
        q_parc(:,j+1) = AttitudeUpdate.singleSamplePARC(...
            q_parc(:,j+1), omega2, omega_sequence(:,i), T);
        q_parc(:,j+1) = AttitudeUpdate.singleSamplePARC(...
            q_parc(:,j+1), omega3, omega_sequence(:,i+1), T);
        q_parc(:,j+1) = AttitudeUpdate.singleSamplePARC(...
            q_parc(:,j+1), omega4, omega_sequence(:,i+2), T);
    else
        q_parc(:,j+1) = AttitudeUpdate.singleSample(...
            q_parc(:,j), omega1, T);
        q_parc(:,j+1) = AttitudeUpdate.singleSample(...
            q_parc(:,j+1), omega2, T);
        q_parc(:,j+1) = AttitudeUpdate.singleSample(...
            q_parc(:,j+1), omega3, T);
        q_parc(:,j+1) = AttitudeUpdate.singleSample(...
            q_parc(:,j+1), omega4, T);
    end
    
    % 计算欧拉角
    euler_single(:,j+1) = quat2euler_ZXY(q_single(:,j+1));
    euler_two(:,j+1) = quat2euler_ZXY(q_two(:,j+1));
    euler_parc(:,j+1) = quat2euler_ZXY(q_parc(:,j+1));
    euler_four(:,j+1) = quat2euler_ZXY(q_four(:,j+1));
    quat_eular(:,j+1) = quat2euler_ZXY(quat(:,j+1));
    j = j + 1;
end

%% 绘制滤波前后加速度对比
figure('Name', '加速度滤波前后对比');

subplot(3,1,1);
plot(t_acc, acc_data_raw(:,1), 'r-', 'LineWidth', 1); hold on;
plot(t_acc, acc_data(:,1), 'b-', 'LineWidth', 1.5);
title('X轴加速度滤波前后对比');
legend('原始', '4Hz低通滤波', 'Location', 'best');
ylabel('加速度 (m/s²)');
grid on;

subplot(3,1,2);
plot(t_acc, acc_data_raw(:,2), 'r-', 'LineWidth', 1); hold on;
plot(t_acc, acc_data(:,2), 'b-', 'LineWidth', 1.5);
title('Y轴加速度滤波前后对比');
legend('原始', '4Hz低通滤波', 'Location', 'best');
ylabel('加速度 (m/s²)');
grid on;

subplot(3,1,3);
plot(t_acc, acc_data_raw(:,3), 'r-', 'LineWidth', 1); hold on;
plot(t_acc, acc_data(:,3), 'b-', 'LineWidth', 1.5);
title('Z轴加速度滤波前后对比');
legend('原始', '4Hz低通滤波', 'Location', 'best');
ylabel('加速度 (m/s²)');
xlabel('时间 (s)');
grid on;

%% 绘制四元数结果
figure('Name', '四元数对比');
subplot(4,1,1);
plot(t_single, q_single(1,:), 'b-', ...
     t_single, q_two(1,:), 'r--', ...
     t_single, q_parc(1,:), 'g-.', ...
     t_single, data.quat_w(1:4:length(omega_sequence)), 'k-.');
title('四元数分量 w');
legend('单子样', '双子样', 'PARC');
grid on;

subplot(4,1,2);
plot(t_single, q_single(2,:), 'b-', ...
     t_single, q_two(2,:), 'r--', ...
     t_single, q_parc(2,:), 'g-.', ...
     t_single, data.quat_x(1:4:length(omega_sequence)), 'k-.');
title('四元数分量 x');
grid on;

subplot(4,1,3);
plot(t_single, q_single(3,:), 'b-', ...
     t_single, q_two(3,:), 'r--', ...
     t_single, q_parc(3,:), 'g-.', ...
     t_single, data.quat_y(1:4:length(omega_sequence)), 'k-.');
title('四元数分量 y');
grid on;

subplot(4,1,4);
plot(t_single, q_single(4,:), 'b-', ...
     t_single, q_two(4,:), 'r--', ...
     t_single, q_parc(4,:), 'g-.', ...
     t_single, data.quat_z(1:4:length(omega_sequence)), 'k-.');
title('四元数分量 z');
grid on;

%% 绘制欧拉角结果（使用各自的时间轴）
figure('Name', '欧拉角对比（含加速度计算）');
subplot(3,1,1);
plot(t_single, (euler_single(1,:)), 'b-', 'DisplayName', '单子样'); hold on;
plot(t_single, (euler_two(1,:)), 'r--', 'DisplayName', '双子样');
plot(t_single, (euler_parc(1,:)), 'g-.', 'DisplayName', 'PARC');
plot(t_single, (euler_four(1,:)), 'k-', 'DisplayName', '四子样');
plot(t_acc, (euler_acc(1,:)), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算(4Hz滤波)');
plot(t_single, data.roll(1:4:length(omega_sequence)), 'c-', 'DisplayName', 'HI原始');
title('Roll角 (deg)');
legend('Location', 'best');
grid on;

subplot(3,1,2);
plot(t_single, (euler_single(2,:)), 'b-', 'DisplayName', '单子样'); hold on;
plot(t_single, (euler_two(2,:)), 'r--', 'DisplayName', '双子样');
plot(t_single, (euler_parc(2,:)), 'g-.', 'DisplayName', 'PARC');
plot(t_single, (euler_four(2,:)), 'k-.', 'DisplayName', '四子样');
plot(t_acc, (euler_acc(2,:)), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算(4Hz滤波)');
plot(t_single, quat_eular(2,:), 'c-', 'DisplayName', 'HI原始');
title('Pitch角 (deg)');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(t_single, (euler_single(3,:)), 'b-', 'DisplayName', '单子样'); hold on;
plot(t_single, (euler_two(3,:)), 'r--', 'DisplayName', '双子样');
plot(t_single, (euler_parc(3,:)), 'g-.', 'DisplayName', 'PARC');
plot(t_single, (euler_four(3,:)), 'k-.', 'DisplayName', '四子样');
plot(t_acc, (euler_acc(3,:)), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算(4Hz滤波)');
plot(t_single, data.yaw(1:4:length(omega_sequence)), 'c-', 'DisplayName', 'HI原始');
title('Yaw角 (deg)');
legend('Location', 'best');
grid on;
xlabel('时间 (s)');

%% 计算欧拉角误差（需要插值对齐时间轴）
% 将加速度计算结果插值到陀螺仪算法的时间轴上
euler_acc_interp = zeros(3, length(t_single));
for i = 1:3
    euler_acc_interp(i,:) = interp1(t_acc, euler_acc(i,:), t_single, 'linear', 'extrap');
end

euler_CH = quat_eular;
euler_err_single = (euler_single -  euler_CH);
euler_err_parc = (euler_parc -  euler_CH);
euler_err_two = (euler_two -  euler_CH);
euler_err_four = (euler_four - euler_CH);
euler_err_acc = (euler_acc_interp - euler_CH);

% 对每个角度分量进行归一化
for i = 1:3
    for j=1:length(euler_err_single)
        if euler_err_single(i,j) > 180
            euler_err_single(i,j) = euler_err_single(i,j)-360;
        end
        if euler_err_parc(i,j) > 180
            euler_err_parc(i,j) = euler_err_parc(i,j) - 360;
        end
        if euler_err_two(i,j) > 180
            euler_err_two(i,j) = euler_err_two(i,j) - 360;
        end
        if euler_err_acc(i,j) > 180
            euler_err_acc(i,j) = euler_err_acc(i,j) - 360;
        end
        if euler_err_single(i,j) < -180
            euler_err_single(i,j) = euler_err_single(i,j) + 360;
        end
        if euler_err_parc(i,j) < -180
            euler_err_parc(i,j) = euler_err_parc(i,j) + 360;
        end
        if euler_err_two(i,j) < -180
            euler_err_two(i,j) = euler_err_two(i,j) + 360;
        end
        if euler_err_acc(i,j) < -180
            euler_err_acc(i,j) = euler_err_acc(i,j) + 360;
        end
    end
end

%% 绘制欧拉角误差（包含加速度计算误差）
figure('Name', '欧拉角误差对比（含加速度计算）');
subplot(3,1,1);
plot(t_single, euler_err_single(1,:), 'b-', 'DisplayName', '单子样误差'); hold on;
plot(t_single, euler_err_two(1,:), 'r-', 'DisplayName', '双子样误差');
plot(t_single, euler_err_parc(1,:), 'g-.', 'DisplayName', 'PARC误差');
plot(t_single, euler_err_four(1,:), 'k-.', 'DisplayName', '四子样误差');
plot(t_single, euler_err_acc(1,:), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算误差(4Hz滤波)');
title('Roll角误差 (deg)');
legend('Location', 'best');
grid on;

subplot(3,1,2);
plot(t_single, euler_err_single(2,:), 'b-', 'DisplayName', '单子样误差'); hold on;
plot(t_single, euler_err_two(2,:), 'r-', 'DisplayName', '双子样误差');
plot(t_single, euler_err_parc(2,:), 'g-.', 'DisplayName', 'PARC误差');
plot(t_single, euler_err_four(2,:), 'k-.', 'DisplayName', '四子样误差');
plot(t_single, euler_err_acc(2,:), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算误差(4Hz滤波)');
title('Pitch角误差 (deg)');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(t_single, euler_err_single(3,:), 'b-', 'DisplayName', '单子样误差'); hold on;
plot(t_single, euler_err_two(3,:), 'r-', 'DisplayName', '双子样误差');
plot(t_single, euler_err_parc(3,:), 'g-.', 'DisplayName', 'PARC误差');
plot(t_single, euler_err_four(3,:), 'k-.', 'DisplayName', '四子样误差');
plot(t_single, euler_err_acc(3,:), 'm:', 'LineWidth', 2, 'DisplayName', '加速度计算误差(4Hz滤波)');
title('Yaw角误差 (deg)');
legend('Location', 'best');
grid on;
xlabel('时间 (s)');

%% 显示时间轴信息
fprintf('时间轴信息:\n');
fprintf('陀螺仪算法时间轴长度: %d 点, 采样频率: %.1f Hz\n', length(t_single), 1/(t_single(2)-t_single(1)));
fprintf('加速度计算时间轴长度: %d 点, 采样频率: %.1f Hz\n', length(t_acc), 1/(t_acc(2)-t_acc(1)));
fprintf('滤波器参数:\n');
fprintf('采样频率: %.1f Hz\n', fs);
fprintf('截止频率: %.1f Hz\n', fc);
fprintf('滤波器阶数: 4阶巴特沃斯\n');
fprintf('滤波方式: 零相位滤波(filtfilt)\n');

%% 加速度转姿态角函数 (ENU坐标系)
function euler = acc2attitude_ENU(acc, yaw)
    % 输入: acc - 加速度向量 [ax, ay, az] (m/s^2)
    %       yaw - 偏航角 (弧度)
    % 输出: euler - 欧拉角 [roll, pitch, yaw] (度)
    
    % 重力在机体坐标系中的表示
    gb = -acc;  % gb = -fb
    
    % 归一化
    gb = gb / norm(gb);
    
    % ENU坐标系下的姿态角计算
    pitch = asind(-gb(2));
    roll = atan2d(gb(1), -gb(3));
    yaw_deg = yaw * 180/pi;  % 转换为度
    
    euler = [roll; pitch; yaw_deg];
end

%% 四元数转欧拉角函数
function euler = quat2euler(q)
    % 输入四元数 q = [w;x;y;z]
    % 输出欧拉角 euler = [roll;pitch;yaw] (弧度)
    
    % 提取四元数分量
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % 计算roll (x轴旋转)
    roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
    
    % 计算pitch (y轴旋转)
    pitch = asin(2*(qw*qy - qz*qx));
    
    % 计算yaw (z轴旋转)
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
    
    euler = [roll; pitch; yaw];
end
%% 四元数转欧拉角函数（Z-X-Y 旋转顺序）
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