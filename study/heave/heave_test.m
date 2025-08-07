clear;
close all;
clc;

%%加载升沉数据
data = readtable("90mm0.2hz-14-14-18.csv");

%%加载数据
GRAVITY = 9.81;%重力加速度
n = height(data);%数据行数


%%
Fs = 100;   % 采样频率
Fc = 0.05;  % 高通截止频率 (Hz),为了同时保证高低频率升沉的效果，应该动态的决定滤波频率
dt = 1/Fs;  % 采样时间
% 归一化截止频率（相对于奈奎斯特频率）
Wn = Fc / (Fs / 2);

%% 设计四阶巴特沃斯高通滤波器 (由二阶改为四阶)
[b_hp, a_hp] = butter(4, Wn, 'high');  % 从2阶改为4阶
t = (0:n-1) * dt; % 时间向量 (秒)
% fvtool(b_hp, a_hp);
%% 生成频率-相位查找表（为MCU优化）
% 定义关键频率点（根据实际需求调整）
freq_points = [0.02 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5];
n_points = length(freq_points);

% 计算这些频率点的相位响应
[h_table, ~] = freqz(b_hp, a_hp, freq_points, Fs);
phase_table = unwrap(angle(h_table))*180/pi;

% 打印C代码格式的查找表
fprintf('\n// 频率-相位查找表 (MCU实现用,理论值)\n');
fprintf('const uint8_t PHASE_TABLE_SIZE = %d;\n', n_points);
fprintf('const float freq_table[%d] = {', n_points);
fprintf('%.2f,', freq_points(1:end-1));
fprintf('%.2f};\n', freq_points(end));

fprintf('const float phase_table[%d] = {', n_points);
fprintf('%.1f,', phase_table(1:end-1));
fprintf('%.1f};\n', phase_table(end));

%% 非线性频率估计器初始化
% 参数设置,调节稳定性和响应速度
f_up = 0.3; % 最大频率 0.3Hz 
frq_est = aranovskiy_freq_est(Fs, f_up);

% 初始化存储变量
log.est_freq = zeros(n,1);

% 初始化
Qb2n = [data.quat_w, data.quat_x,data.quat_y,data.quat_z];%加载四元数
acc_b = [data.acc_x, data.acc_y, data.acc_z] * GRAVITY;%加载加速度数据
gyro_b = [data.gyr_x, data.gyr_y, data.gyr_z] * 3.1415 / 180;
r = [0 -0.130 0];
acc_bm = acc_b;
a_w_b = [0 0 0;diff(gyro_b)*Fs];%%
for i=1:length(acc_b)
    acc_bm(i,:) = acc_b(i,:) - (cross(a_w_b(i,:),r) + cross(gyro_b(i,:),(cross(gyro_b(i,:),r))));
end
% 调用body2nav函数计算导航系下加速度
[acc_n, raw_acc_n] = body2nav(Qb2n, acc_bm, GRAVITY);
figure;
plot(acc_bm(:,3));hold on;plot(acc_b(:,3))
%% 三次高通滤波实现升沉估计 - 使用for循环实现
% 初始化状态变量
hp_acc_n = zeros(n,1);
vel = zeros(n,1);
hp_vel = zeros(n,1);
heave = zeros(n,1);
hp_heave = zeros(n,1);

% 滤波器状态变量 - 每个滤波器需要保存前四个输入和输出 (由于是四阶滤波器)
x1 = zeros(5,1); y1 = zeros(5,1); % 第一次高通滤波
x2 = zeros(5,1); y2 = zeros(5,1); % 第二次高通滤波
x3 = zeros(5,1); y3 = zeros(5,1); % 第三次高通滤波

% 使用for循环逐点处理
for i = 1:n
    % 1. 第一次高通滤波 - 处理加速度
    % 更新滤波器状态 (四阶滤波器需要更多状态)
    x1 = [acc_n(i); x1(1:4)]; 
    y_new = b_hp(1)*x1(1) + b_hp(2)*x1(2) + b_hp(3)*x1(3) + b_hp(4)*x1(4) + b_hp(5)*x1(5) - ...
            a_hp(2)*y1(1) - a_hp(3)*y1(2) - a_hp(4)*y1(3) - a_hp(5)*y1(4);
    y1 = [y_new; y1(1:4)];
    hp_acc_n(i) = y_new;
    
    % 2. 积分得到速度
    if i == 1
        vel(i) = 0;  % 初始速度为0
    else
        vel(i) = vel(i-1) + hp_acc_n(i) * dt;
    end
    
    % 3. 第二次高通滤波 - 处理速度
    % 更新滤波器状态 (四阶滤波器)
    x2 = [vel(i); x2(1:4)];
    y_new = b_hp(1)*x2(1) + b_hp(2)*x2(2) + b_hp(3)*x2(3) + b_hp(4)*x2(4) + b_hp(5)*x2(5) - ...
            a_hp(2)*y2(1) - a_hp(3)*y2(2) - a_hp(4)*y2(3) - a_hp(5)*y2(4);
    y2 = [y_new; y2(1:4)];
    hp_vel(i) = y_new;
    
    % 4. 积分得到位移
    if i == 1
        heave(i) = 0;  % 初始位移为0
    else
        heave(i) = heave(i-1) + hp_vel(i) * dt;
    end
    
%     % 5. 第三次高通滤波 - 处理位移
%     % 更新滤波器状态 (四阶滤波器)
%     x3 = [heave(i); x3(1:4)];
%     y_new = b_hp(1)*x3(1) + b_hp(2)*x3(2) + b_hp(3)*x3(3) + b_hp(4)*x3(4) + b_hp(5)*x3(5) - ...
%             a_hp(2)*y3(1) - a_hp(3)*y3(2) - a_hp(4)*y3(3) - a_hp(5)*y3(4);
%     y3 = [y_new; y3(1:4)];
%     hp_heave(i) = y_new;

    hp_heave(i) = heave(i);
    
    % 6. 非线性正弦频率估计
    current_freq = frq_est.update(hp_heave(i));
    log.est_freq(i) = current_freq;
end

%% 创建带通滤波器用于参考 - 避免直接积分的漂移问题
% 使用主频率附近的带通滤波器来获取"理想"的积分结果
est_freq = median(log.est_freq(floor(n/2):end)); % 使用后半段数据的频率估计
if est_freq < 0.05 % 如果频率估计不可靠
    est_freq = 0.2; % 使用默认值
end

% 根据估计的频率选择最接近的频率点
[~, closest_idx] = min(abs(freq_points - est_freq));
closest_freq = freq_points(closest_idx);
phase_to_correct = phase_table(closest_idx);
if phase_to_correct < 0
    phase_to_correct = phase_to_correct + 360;
end
fprintf('\n=== 全通滤波器相位校正 ===\n');
fprintf('检测主频率: %.3f Hz, 最接近查找表频率: %.3f Hz\n', est_freq, closest_freq);
fprintf('需要校正的相位: %.1f 度\n', phase_to_correct);

% 计算归一化角频率
omega_0 = 2 * pi * closest_freq / Fs;

% 计算所需的相位补偿（取负值，因为我们要补偿）
phase_rad = 2 * phase_to_correct * pi / 180;

% 计算全通滤波器系数 - 为四阶滤波器设计级联的二阶全通滤波器
% 将总相位校正分为两部分，每个二阶滤波器校正一半
phase_rad_half = phase_rad;

% 计算一阶全通滤波器的alpha值
alpha = sin((omega_0 - phase_rad_half)/2) / sin((omega_0 + phase_rad_half)/2);

% 确保滤波器稳定性
if abs(alpha) >= 1
    fprintf('警告: 计算的alpha1值 %.6f 会导致不稳定滤波器\n', alpha);
    alpha = sign(alpha) * 0.99; % 限制在稳定范围内
    fprintf('已调整为: %.6f\n', alpha);
end

fprintf('全通滤波器系数 alpha: %.6f\n', alpha);

% 创建两个级联的二阶全通滤波器系数
b_ap = [alpha, 1];
a_ap = [1, alpha];

% 应用级联全通滤波器到 hp_heave 进行相位校正
hp_heave_corrected = filter(b_ap, a_ap, hp_heave);

%% 相位误差分析 - 使用零相位滤波作为参考
% 使用filtfilt实现零相位滤波作为理想参考
hp_acc_n_zero = filtfilt(b_hp, a_hp, acc_n);
vel_zero = [0; cumsum(hp_acc_n_zero(2:end)*dt)];
hp_vel_zero = filtfilt(b_hp, a_hp, vel_zero);
heave_zero = [0; cumsum(hp_vel_zero(2:end)*dt)];
hp_heave_zero = filtfilt(b_hp, a_hp, heave_zero);

% 窗口化处理
N = length(acc_n);
window = hann(N);
hp_heave_windowed = hp_heave .* window;
hp_heave_zero_windowed = hp_heave_zero .* window;
hp_heave_corrected_windowed = hp_heave_corrected .* window;

% FFT分析
freq = (0:N-1)*(Fs/N);
Y_hp_heave = fft(hp_heave_windowed);
Y_hp_heave_zero = fft(hp_heave_zero_windowed);
Y_hp_heave_corrected = fft(hp_heave_corrected_windowed);

% 找到主要频率成分
[~, peak_indices] = findpeaks(abs(Y_hp_heave_zero(1:floor(N/2))), 'MinPeakHeight', max(abs(Y_hp_heave_zero(1:floor(N/2))))/10);
if ~isempty(peak_indices)
    valid_peaks = peak_indices(freq(peak_indices) > 0.05); % 排除过低频率
    if ~isempty(valid_peaks)
        est_freq_idx = valid_peaks(1); % 使用第一个有效峰值
        est_freq = freq(est_freq_idx);
        
        % 计算三种升沉信号的相位
        phase_hp_heave = angle(Y_hp_heave(est_freq_idx));
        phase_hp_heave_zero = angle(Y_hp_heave_zero(est_freq_idx));
        phase_hp_heave_corrected = angle(Y_hp_heave_corrected(est_freq_idx));
        
        % 计算实际相位差 (高通滤波相位减去零相位)
        phase_diff_hp_vs_zero = (phase_hp_heave - phase_hp_heave_zero) * 180/pi;
        phase_diff_hp_vs_zero = mod(phase_diff_hp_vs_zero + 180, 360) - 180;
        
        % 计算校正后相位差 (校正后相位减去零相位)
        phase_diff_corrected_vs_zero = (phase_hp_heave_corrected - phase_hp_heave_zero) * 180/pi;
        phase_diff_corrected_vs_zero = mod(phase_diff_corrected_vs_zero + 180, 360) - 180;
        
        % 输出分析结果
        fprintf('\n=== 相位误差分析结果 ===\n');
        fprintf('主频率: %.3f Hz\n', est_freq);
        fprintf('高通滤波vs零相位相位差: %.2f 度\n', phase_diff_hp_vs_zero);
        fprintf('校正后vs零相位相位差: %.2f 度\n', phase_diff_corrected_vs_zero);
        fprintf('相位校正改善: %.2f 度\n', abs(phase_diff_hp_vs_zero) - abs(phase_diff_corrected_vs_zero));
    end
end


%% 绘图
figure('Name', '升沉估计分析 - 实时处理与相位校正 (四阶滤波器)', 'Position', [50, 50, 1200, 900]);

%% 1. 三次高通滤波过程 - 左上
subplot(3, 2, 1);
plot(t, acc_n, 'Color', [0.7 0.7 0.7], 'LineWidth', 1, 'DisplayName', '原始加速度');
hold on;
plot(t, hp_acc_n, 'b', 'LineWidth', 1.2, 'DisplayName', '高通滤波后');
xlabel('时间 (s)'); ylabel('加速度 (m/s²)');
title('1. 加速度信号滤波 (四阶)');
legend('Location', 'best');
grid on;

%% 2. 速度积分结果(经过高通滤波) - 右上
subplot(3, 2, 2);
plot(t, vel, 'b', 'LineWidth', 1.2, 'DisplayName', '高通滤波后');
xlabel('时间 (s)'); ylabel('速度 (m/s)');
title('2. 高通滤波后的速度积分结果 (四阶)');
grid on;

%% 3. 频率估计曲线 - 左中
subplot(3, 2, 3);
plot(t, log.est_freq, 'LineWidth', 1.5, 'Color', [0.2 0.6 0.2]);
xlabel('时间 (s)'); ylabel('频率估计 (Hz)');
title('3. 波浪频率估计');
ylim([0, f_up*1.2]); % 设置合理的Y轴范围
grid on;

%% 4. 最终升沉结果对比 - 右中
subplot(3, 2, 4);
plot(t, hp_heave, 'b', 'LineWidth', 1.2, 'DisplayName', '三次高通滤波');
hold on;
if exist('hp_heave_corrected', 'var')
    plot(t, hp_heave_corrected, 'g', 'LineWidth', 1.2, 'DisplayName', '相位校正');
end
plot(t, hp_heave_zero, 'r--', 'LineWidth', 1.0, 'DisplayName', '零相位参考');
xlabel('时间 (s)'); ylabel('位移 (m)');
title(sprintf('4. 升沉估计结果 (四阶滤波器)'));
legend('Location', 'best');
grid on;

%% 5. 相位校正前后对比 - 左下
subplot(3, 2, 5);
% 选择数据的一个有代表性的片段进行放大显示
segment_start = floor(n/2);  % 从中间开始
segment_length = min(500, floor(n/4));  % 取适当长度
segment_idx = segment_start:(segment_start+segment_length-1);

plot(t(segment_idx), hp_heave(segment_idx), 'b', 'LineWidth', 1.2, 'DisplayName', '滤波前');
hold on;
plot(t(segment_idx), hp_heave_corrected(segment_idx), 'g', 'LineWidth', 1.2, 'DisplayName', '相位校正后');
plot(t(segment_idx), hp_heave_zero(segment_idx), 'r--', 'LineWidth', 1.0, 'DisplayName', '零相位参考');
xlabel('时间 (s)'); ylabel('位移 (m)');
title('5. 相位校正效果对比 (局部放大)');
legend('Location', 'best');
grid on;

%% 6. 直接积分的速度和位移(有漂移) - 右下
subplot(3, 2, 6);
yyaxis left
plot(t, vel, 'Color', [0.8 0.4 0.2], 'LineWidth', 1.5, 'DisplayName', '直接积分速度');
ylabel('速度 (m/s)');

yyaxis right
plot(t, heave, 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5, 'DisplayName', '直接积分位移');
ylabel('位移 (m)');

xlabel('时间 (s)');
title('6. 未经高通滤波的直接积分结果(显示漂移)');
legend('Location', 'best');
grid on;

% 调整整体布局
set(gcf, 'Color', 'w');
set(findall(gcf,'-property','FontSize'),'FontSize', 11);

% 添加滤波器阶数信息
sgtitle('升沉估计分析 - 四阶巴特沃斯滤波器实现', 'FontSize', 14);
