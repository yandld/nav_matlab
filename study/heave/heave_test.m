clear;
close all;
clc;

%%加载升沉数据
data = readtable("90mm0.5hz-14-10-25.csv");

% 90mm0.1hz-14-18-20
% 90mm0.2hz-14-14-18
% 90mm0.5hz-14-10-25
% line_0.1Hz_106mm
% line_0.1Hz_106mm_B
% 100s_110mm_0.2Hz_sin5
% 50s_90mm幅值_0.2hz_sin波形
% huizhou_400mm-14-14-47

% ch_108mm_0.2Hz-16-09-46


%%加载数据
GRAVITY = 9.81;%重力加速度
n = height(data);%数据行数


%%
Fs = 100;   % 采样频率
Fc = 0.03;  % 高通截止频率 (Hz),为了同时保证高低频率升沉的效果，应该动态的决定滤波频率
dt = 1/Fs;  % 采样时间
% 归一化截止频率（相对于奈奎斯特频率）
Wn = Fc / (Fs / 2);

%% 设计二阶巴特沃斯高通滤波器 
[b_hp, a_hp] = butter(2, Wn, 'high');
t = (0:n-1) * dt; % 时间向量 (秒)

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
vel = zeros(n,1);
heave = zeros(n,1);

%kf初始化
Qb2n = [data.quat_w, data.quat_x,data.quat_y,data.quat_z];%加载四元数
acc_b = [data.acc_x, data.acc_y, data.acc_z] * GRAVITY;%加载加速度数据

% 调用body2nav函数计算导航系下加速度
[acc_n, raw_acc_n] = body2nav(Qb2n, acc_b, GRAVITY);

%% 三次高通滤波实现升沉估计
% 1. 第一次高通滤波 - 处理加速度
hp_acc_n = filter(b_hp, a_hp, acc_n);

% 2. 积分得到速度 - 向量化实现
vel = [0; cumsum(hp_acc_n(2:end)*dt)];

% 3. 第二次高通滤波 - 处理速度
hp_vel = filter(b_hp, a_hp, vel);

% 4. 积分得到位移 - 向量化实现
heave = [0; cumsum(hp_vel(2:end)*dt)];

% 5. 第三次高通滤波 - 处理位移，得到最终升沉结果
hp_heave = filter(b_hp, a_hp, heave);

% 非线性正弦频率估计
for i = 1:n
    current_freq = frq_est.update(hp_heave(i));
    log.est_freq(i) = current_freq;
end

%% 创建带通滤波器用于参考 - 避免直接积分的漂移问题
% 使用主频率附近的带通滤波器来获取"理想"的积分结果
est_freq = median(log.est_freq(floor(n/2):end)); % 使用后半段数据的频率估计
if est_freq < 0.05 % 如果频率估计不可靠
    est_freq = 0.2; % 使用默认值
end


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

% FFT分析
freq = (0:N-1)*(Fs/N);
Y_hp_heave = fft(hp_heave_windowed);
Y_hp_heave_zero = fft(hp_heave_zero_windowed);

% 找到主要频率成分
[~, peak_indices] = findpeaks(abs(Y_hp_heave_zero(1:floor(N/2))), 'MinPeakHeight', max(abs(Y_hp_heave_zero(1:floor(N/2))))/10);
if ~isempty(peak_indices)
    valid_peaks = peak_indices(freq(peak_indices) > 0.05); % 排除过低频率
    if ~isempty(valid_peaks)
        main_freq_idx = valid_peaks(1); % 使用第一个有效峰值
        main_freq = freq(main_freq_idx);
        
        % 计算相位
        phase_hp_heave = angle(Y_hp_heave(main_freq_idx));
        phase_hp_heave_zero = angle(Y_hp_heave_zero(main_freq_idx));
        
        % 计算实际相位差 (滤波相位减去零相位)
        actual_phase_diff = (phase_hp_heave - phase_hp_heave_zero) * 180/pi;
        actual_phase_diff = mod(actual_phase_diff + 180, 360) - 180;
        
        
        % 输出分析结果
        fprintf('\n=== 相位误差分析结果 ===\n');
        fprintf('主频率: %.3f Hz\n', main_freq);
        fprintf('实际相位差(滤波vs零相位): %.2f 度\n', actual_phase_diff);
    end
end


%% 新增相位对比图 - 简化版
figure('Name', '相位对比分析', 'Position', [100, 100, 900, 400]);

% 1. 时域信号对比 - 零相位vs普通滤波
subplot(1,2,1);
plot(t, hp_heave, 'b', 'LineWidth', 1.5, 'DisplayName', '三次高通滤波');
hold on;
plot(t, hp_heave_zero, 'r', 'LineWidth', 1.5, 'DisplayName', '零相位滤波');
xlabel('时间 (s)');
ylabel('升沉 (m)');
title('升沉估计对比');
legend('Location', 'best');
grid on;

% 2. 窄带滤波后的相位对比
subplot(1,2,2);
if exist('main_freq', 'var')
    % 使用带通滤波获取窄带信号以便清晰显示相位
    [b_narrow, a_narrow] = butter(2, [main_freq-0.02 main_freq+0.02]/(Fs/2), 'bandpass');
    narrow_hp = filter(b_narrow, a_narrow, hp_heave);
    narrow_hp_zero = filtfilt(b_narrow, a_narrow, hp_heave_zero);
    
    % 归一化
    norm_hp = narrow_hp / max(abs(narrow_hp));
    norm_hp_zero = narrow_hp_zero / max(abs(narrow_hp_zero));
    
    % 选择一段稳定的数据进行显示
    start_idx = floor(n/2);
    window_size = min(round(5*Fs/main_freq), n-start_idx); % 显示约5个周期
    plot_range = start_idx:(start_idx+window_size);
    
    plot(t(plot_range), norm_hp(plot_range), 'b', 'LineWidth', 1.5, 'DisplayName', '三次高通滤波');
    hold on;
    plot(t(plot_range), norm_hp_zero(plot_range), 'r', 'LineWidth', 1.5, 'DisplayName', '零相位滤波');
    xlabel('时间 (s)');
    ylabel('归一化幅值');
    title(sprintf('窄带滤波后相位对比 (%.3f Hz)\n相位差: %.1f°', main_freq, actual_phase_diff));
    legend('Location', 'best');
    grid on;
end

set(gcf, 'Color', 'w');
set(findall(gcf,'-property','FontSize'),'FontSize', 11);

