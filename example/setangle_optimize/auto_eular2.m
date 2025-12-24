%% 基于数值优化的欧拉角偏置估计 - 带验证功能
% 使用fminunc在欧拉角空间优化偏置
% 目标: 最小化相对旋转的残差
clear; clc; close all;

%% ========================================================================
%% 验证模式设置
%% ========================================================================
ENABLE_VALIDATION = true;  % 🔧 设为true启用验证模式

% 人工添加的已知偏置（ground truth）

SETANGLE = -0 * [0.0, 0.2694,-1.1328];%抵消实际安装误差
GROUND_TRUTH_BIAS_DEG = SETANGLE + 0 * [0.0, 1 , 2];  % 模拟安装差输入[roll, pitch, yaw] in degrees,貌似目前只能估计俯仰横滚
GROUND_TRUTH_BIAS_RAD = GROUND_TRUTH_BIAS_DEG * pi/180;

fprintf('========================================\n');
fprintf('验证模式: %s\n', mat2str(ENABLE_VALIDATION));
if ENABLE_VALIDATION
    fprintf('已知偏置 (Ground Truth):\n');
    fprintf('  Roll:  %.4f° (%.6f rad)\n', GROUND_TRUTH_BIAS_DEG(1), GROUND_TRUTH_BIAS_RAD(1));
    fprintf('  Pitch: %.4f° (%.6f rad)\n', GROUND_TRUTH_BIAS_DEG(2), GROUND_TRUTH_BIAS_RAD(2));
    fprintf('  Yaw:   %.4f° (%.6f rad)\n', GROUND_TRUTH_BIAS_DEG(3), GROUND_TRUTH_BIAS_RAD(3));
end
fprintf('========================================\n\n');

%% 参数设置
TIME_OFFSET_FILE1 = 1.23; % 文件1时间偏移 (秒)。 注：此代码数据非同时录制数据，时间同会存在问题，相对欧拉角最终会比实际偏大

%% 读取数据
fprintf('正在读取CSV文件...\n');
data1 = readtable('HI91-1-092711.csv');
data2 = readtable('HI91-2-092710.csv');

%% 提取四元数数据 [w, x, y, z]
quat1 = [data1.quat_w, data1.quat_x, data1.quat_y, data1.quat_z];
quat2 = [data2.quat_w, data2.quat_x, data2.quat_y, data2.quat_z];

%% 提取时间戳并对齐
time1_s = (data1.sys_time - data1.sys_time(1)) / 1000 + TIME_OFFSET_FILE1;
time2_s = (data2.sys_time - data2.sys_time(1)) / 1000;

time_start = max(time1_s(1), time2_s(1));
time_end = min(time1_s(end), time2_s(end));

idx1 = find(time1_s >= time_start & time1_s <= time_end);
idx2 = find(time2_s >= time_start & time2_s <= time_end);

%% 插值对齐
time_common = time2_s(idx2);
quat1_interp = interp1(time1_s(idx1), quat1(idx1, :), time_common, 'linear');
quat2_final = quat2(idx2, :);

valid_idx = ~any(isnan(quat1_interp), 2);
time_common = time_common(valid_idx);
quat1_interp = quat1_interp(valid_idx, :);
quat2_final = quat2_final(valid_idx, :);

fprintf('有效数据点: %d | 时间偏移: %.2fs\n', size(quat1_interp, 1), TIME_OFFSET_FILE1);

%% ========================================================================
%% 🧪 验证模式: 人工添加偏置到quat2
%% ========================================================================

if ENABLE_VALIDATION
    fprintf('\n🧪 验证模式: 向quat2添加已知偏置...\n');
    
    % 将quat2转为欧拉角
    euler2_original = q2att_ENU(quat2_final);
    
    % 添加已知偏置
    euler2_biased = euler2_original + repmat(GROUND_TRUTH_BIAS_RAD, size(euler2_original, 1), 1);
    
    % 转回四元数
    quat2_biased = att2q_ENU(euler2_biased);
    quat2_biased = quat2_biased ./ sqrt(sum(quat2_biased.^2, 2));
    
    % 用带偏置的数据替换原始quat2
    quat2_final = quat2_biased;
    
    fprintf('✅ 已添加偏置到quat2\n');
end

%% ========================================================================
%% 计算原始相对旋转（带偏置的）
%% ========================================================================

quat1_conj = qconj_func(quat1_interp);
q_relative_orig = qmul_func(quat1_conj, quat2_final);
q_relative_orig = q_relative_orig ./ sqrt(sum(q_relative_orig.^2, 2));

euler_relative_orig = q2att_ENU(q_relative_orig);
euler_relative_orig_deg = euler_relative_orig * 180/pi;

fprintf('\n原始相对欧拉角统计 (带偏置):\n');
fprintf('  Roll:  Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_orig_deg(:,1)), std(euler_relative_orig_deg(:,1)));
fprintf('  Pitch: Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_orig_deg(:,2)), std(euler_relative_orig_deg(:,2)));
fprintf('  Yaw:   Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_orig_deg(:,3)), std(euler_relative_orig_deg(:,3)));

%% ========================================================================
%% 欧拉角空间优化: 求解最优偏置
%% ========================================================================

fprintf('\n========================================\n');
fprintf('欧拉角空间数值优化\n');
fprintf('========================================\n');

% 初始猜测: 零偏置
delta_euler_init = [0; 0; 0]; % [roll; pitch; yaw] in radians

% 优化选项
options = optimoptions('fminunc', ...
    'Algorithm', 'quasi-newton', ...
    'Display', 'iter', ...
    'MaxIterations', 100, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

% 定义目标函数: 最小化相对旋转的标准差
objective_func = @(delta) compute_euler_cost(delta, quat1_interp, quat2_final);

% 执行优化
fprintf('\n开始优化...\n');
tic;
[delta_euler_opt, fval, exitflag, output] = fminunc(objective_func, delta_euler_init, options);
opt_time = toc;

fprintf('\n优化完成!\n');
fprintf('耗时: %.2f 秒\n', opt_time);
fprintf('迭代次数: %d\n', output.iterations);
fprintf('函数评估次数: %d\n', output.funcCount);
fprintf('最终损失值: %.6f°\n', fval);

fprintf('\n最优偏置 (弧度):\n');
fprintf('  Roll:  %.6f rad\n', delta_euler_opt(1));
fprintf('  Pitch: %.6f rad\n', delta_euler_opt(2));
fprintf('  Yaw:   %.6f rad\n', delta_euler_opt(3));

delta_euler_opt_deg = delta_euler_opt * 180/pi;
fprintf('\n最优偏置 (度):\n');
fprintf('  Roll:  %.4f°\n', delta_euler_opt_deg(1));
fprintf('  Pitch: %.4f°\n', delta_euler_opt_deg(2));
fprintf('  Yaw:   %.4f°\n', delta_euler_opt_deg(3));

%% ========================================================================
%% 🎯 验证结果对比
%% ========================================================================

if ENABLE_VALIDATION
    fprintf('\n========================================\n');
    fprintf('🎯 验证结果对比\n');
    fprintf('========================================\n');
    
    error_deg = delta_euler_opt_deg - GROUND_TRUTH_BIAS_DEG;
    error_pct = abs(error_deg ./ GROUND_TRUTH_BIAS_DEG) * 100;
    
    fprintf('\n%-10s | %12s | %12s | %12s | %12s\n', '轴', 'Ground Truth', '优化结果', '误差', '相对误差');
    fprintf('-----------|--------------|--------------|--------------|------------\n');
    fprintf('%-10s | %10.4f° | %10.4f° | %10.4f° | %10.2f%%\n', ...
        'Roll', GROUND_TRUTH_BIAS_DEG(1), delta_euler_opt_deg(1), error_deg(1), error_pct(1));
    fprintf('%-10s | %10.4f° | %10.4f° | %10.4f° | %10.2f%%\n', ...
        'Pitch', GROUND_TRUTH_BIAS_DEG(2), delta_euler_opt_deg(2), error_deg(2), error_pct(2));
    fprintf('%-10s | %10.4f° | %10.4f° | %10.4f° | %10.2f%%\n', ...
        'Yaw', GROUND_TRUTH_BIAS_DEG(3), delta_euler_opt_deg(3), error_deg(3), error_pct(3));
    
    rmse = sqrt(mean(error_deg.^2));
    max_error = max(abs(error_deg));
    
    fprintf('\n统计指标:\n');
    fprintf('  RMSE (均方根误差): %.4f°\n', rmse);
    fprintf('  最大误差:          %.4f°\n', max_error);
    
    % 判断验证是否成功
    tolerance_deg = 0.01;  % 容差 0.01°
    if max_error < tolerance_deg
        fprintf('\n✅ 验证成功! 最大误差 %.4f° < 容差 %.4f°\n', max_error, tolerance_deg);
    else
        fprintf('\n⚠️  验证警告: 最大误差 %.4f° > 容差 %.4f°\n', max_error, tolerance_deg);
    end
end

%% ========================================================================
%% 应用最优偏置
%% ========================================================================

% 将File2的四元数转为欧拉角
euler2_from_quat = q2att_ENU(quat2_final);

% 应用最优偏置（在欧拉角空间直接相减，因为我们要去除偏置）
euler2_corrected = euler2_from_quat - delta_euler_opt';

% 转回四元数
quat2_corrected = att2q_ENU(euler2_corrected);
quat2_corrected = quat2_corrected ./ sqrt(sum(quat2_corrected.^2, 2));

% 计算修正后的相对旋转
q_relative_corrected = qmul_func(quat1_conj, quat2_corrected);
q_relative_corrected = q_relative_corrected ./ sqrt(sum(q_relative_corrected.^2, 2));

euler_relative_corrected = q2att_ENU(q_relative_corrected);
euler_relative_corrected_deg = euler_relative_corrected * 180/pi;

fprintf('\n修正后相对欧拉角统计:\n');
fprintf('  Roll:  Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_corrected_deg(:,1)), std(euler_relative_corrected_deg(:,1)));
fprintf('  Pitch: Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_corrected_deg(:,2)), std(euler_relative_corrected_deg(:,2)));
fprintf('  Yaw:   Mean=%.4f°, Std=%.4f°\n', mean(euler_relative_corrected_deg(:,3)), std(euler_relative_corrected_deg(:,3)));

%% ========================================================================
%% 优化总结
%% ========================================================================

fprintf('\n========================================\n');
fprintf('优化总结\n');
fprintf('========================================\n');
fprintf('标准差改善:\n');
std_orig = [std(euler_relative_orig_deg(:,1)), std(euler_relative_orig_deg(:,2)), std(euler_relative_orig_deg(:,3))];
std_corrected = [std(euler_relative_corrected_deg(:,1)), std(euler_relative_corrected_deg(:,2)), std(euler_relative_corrected_deg(:,3))];
euler_labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    improvement_pct = (std_orig(i) - std_corrected(i)) / std_orig(i) * 100;
    fprintf('  %s: %.4f° → %.4f° (改善 %.2f%%)\n', ...
        euler_labels{i}, std_orig(i), std_corrected(i), improvement_pct);
end

%% ========================================================================
%% 可视化对比（验证模式）
%% ========================================================================

if ENABLE_VALIDATION
    figure('Name', '验证结果对比', 'Position', [100, 100, 1400, 500]);
    
    % 子图1: 偏置对比
    subplot(1, 3, 1);
    x_pos = 1:3;
    bar_width = 0.35;
    bar(x_pos - bar_width/2, GROUND_TRUTH_BIAS_DEG, bar_width, 'FaceColor', 'b', 'DisplayName', 'Ground Truth');
    hold on;
    bar(x_pos + bar_width/2, delta_euler_opt_deg, bar_width, 'FaceColor', 'r', 'DisplayName', '优化结果');
    grid on;
    set(gca, 'XTick', x_pos, 'XTickLabel', euler_labels);
    ylabel('偏置角度 (°)', 'FontSize', 12);
    title('偏置对比', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    
    % 添加数值标签
    for i = 1:3
        text(x_pos(i) - bar_width/2, GROUND_TRUTH_BIAS_DEG(i), sprintf('%.3f°', GROUND_TRUTH_BIAS_DEG(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
        text(x_pos(i) + bar_width/2, delta_euler_opt_deg(i), sprintf('%.3f°', delta_euler_opt_deg(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
    
    % 子图2: 误差分析
    subplot(1, 3, 2);
    error_deg = delta_euler_opt_deg - GROUND_TRUTH_BIAS_DEG;
    bar(error_deg, 'FaceColor', [0.8 0.4 0.2]);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 1.5);
    set(gca, 'XTick', x_pos, 'XTickLabel', euler_labels);
    ylabel('误差 (°)', 'FontSize', 12);
    title(sprintf('恢复误差 (RMSE=%.4f°)', sqrt(mean(error_deg.^2))), 'FontSize', 14, 'FontWeight', 'bold');
    
    % 添加数值标签
    for i = 1:3
        if error_deg(i) > 0
            valign = 'bottom';
        else
            valign = 'top';
        end
        text(i, error_deg(i), sprintf('%.4f°', error_deg(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', valign, 'FontSize', 10, 'FontWeight', 'bold');
    end
    
    % 子图3: 标准差改善
    subplot(1, 3, 3);
    bar(x_pos - bar_width/2, std_orig, bar_width, 'FaceColor', 'b', 'DisplayName', '带偏置');
    hold on;
    bar(x_pos + bar_width/2, std_corrected, bar_width, 'FaceColor', 'g', 'DisplayName', '修正后');
    grid on;
    set(gca, 'XTick', x_pos, 'XTickLabel', euler_labels);
    ylabel('标准差 (°)', 'FontSize', 12);
    title('相对旋转标准差改善', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    
    % 添加数值标签
    for i = 1:3
        text(x_pos(i) - bar_width/2, std_orig(i), sprintf('%.3f°', std_orig(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
        text(x_pos(i) + bar_width/2, std_corrected(i), sprintf('%.3f°', std_corrected(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
end

%% ========================================================================
%% 新增：相对欧拉角分布对比图
%% ========================================================================

fprintf('\n生成相对欧拉角分布对比图...\n');

figure('Name', '相对欧拉角分布对比', 'Position', [100, 100, 1600, 900]);

for i = 1:3
    % 时间序列（放大显示）
    subplot(3, 2, 2*i-1);
    hold on; grid on;
    
    plot(time_common, euler_relative_orig_deg(:, i), 'r-', 'LineWidth', 1.0, 'DisplayName', '修正前');
    plot(time_common, euler_relative_corrected_deg(:, i), 'g-', 'LineWidth', 1.0, 'DisplayName', '修正后');
    yline(0, 'k--', 'LineWidth', 1.5);
    
    xlabel('时间 (s)', 'FontSize', 10);
    ylabel(['相对' euler_labels{i} ' (°)'], 'FontSize', 10);
    title(sprintf('相对%s 时间序列', euler_labels{i}), 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 9);
    
    % 分布直方图
    subplot(3, 2, 2*i);
    hold on; grid on;
    
    histogram(euler_relative_orig_deg(:, i), 50, 'FaceColor', 'r', 'EdgeColor', 'k', ...
        'FaceAlpha', 0.6, 'DisplayName', '修正前');
    histogram(euler_relative_corrected_deg(:, i), 50, 'FaceColor', 'g', 'EdgeColor', 'k', ...
        'FaceAlpha', 0.6, 'DisplayName', '修正后');
    xline(0, 'k--', 'LineWidth', 2, 'DisplayName', '理想值 (0°)');
    
    xlabel(['相对' euler_labels{i} ' (°)'], 'FontSize', 10);
    ylabel('频数', 'FontSize', 10);
    title(sprintf('相对%s 分布 | Std: %.3f° → %.3f°', euler_labels{i}, ...
        std(euler_relative_orig_deg(:, i)), std(euler_relative_corrected_deg(:, i))), ...
        'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    set(gca, 'FontSize', 9);
end

sgtitle('相对欧拉角分布对比（修正前后）', 'FontSize', 15, 'FontWeight', 'bold');

fprintf('\n分析完成!\n');

%% ========================================================================
%% 函数定义
%% ========================================================================

%% 目标函数: 欧拉角空间优化
function cost = compute_euler_cost(delta_euler, quat1, quat2)
    % delta_euler: [roll; pitch; yaw] in radians
    % 在欧拉角空间直接相减偏置（去除偏置）
    
    % 将quat2转为欧拉角
    euler2 = q2att_ENU(quat2);
    
    % 应用偏置（欧拉角空间的线性减法）
    euler2_corrected = euler2 - delta_euler';
    
    % 转回四元数
    quat2_corrected = att2q_ENU(euler2_corrected);
    quat2_corrected = quat2_corrected ./ sqrt(sum(quat2_corrected.^2, 2));
    
    % 计算相对旋转
    quat1_conj = qconj_func(quat1);
    q_relative = qmul_func(quat1_conj, quat2_corrected);
    q_relative = q_relative ./ sqrt(sum(q_relative.^2, 2));
    
    % 转为欧拉角
    euler_relative = q2att_ENU(q_relative);
    euler_relative_deg = euler_relative * 180/pi;
    
    % 目标: 最小化三个轴的标准差之和
    cost = std(euler_relative_deg(:,1)) + ...
           std(euler_relative_deg(:,2)) + ...
           std(euler_relative_deg(:,3));
end

%% 四元数共轭
function qj = qconj_func(q)
    qj = [q(:,1), -q(:,2), -q(:,3), -q(:,4)];
end

%% 四元数乘法
function r = qmul_func(q1, q2)
    w1 = q1(:,1); x1 = q1(:,2); y1 = q1(:,3); z1 = q1(:,4);
    w2 = q2(:,1); x2 = q2(:,2); y2 = q2(:,3); z2 = q2(:,4);
    
    r = zeros(size(q1));
    r(:,1) = w1.*w2 - x1.*x2 - y1.*y2 - z1.*z2;
    r(:,2) = w1.*x2 + x1.*w2 + y1.*z2 - z1.*y2;
    r(:,3) = w1.*y2 + y1.*w2 + z1.*x2 - x1.*z2;
    r(:,4) = w1.*z2 + z1.*w2 + x1.*y2 - y1.*x2;
end

%% atan2近似
function res = atan2_approx(y, x)
    a1  =  0.99997726;
    a3  = -0.33262347;
    a5  =  0.19354346;
    a7  = -0.11643287;
    a9  =  0.05265332;
    a11 = -0.01172120;
    
    swap = abs(x) < abs(y);
    atan_input = zeros(size(x));
    atan_input(swap) = x(swap) ./ y(swap);
    atan_input(~swap) = y(~swap) ./ x(~swap);
    
    x_sq = atan_input.^2;
    res = atan_input .* (a1 + x_sq .* (a3 + x_sq .* (a5 + x_sq .* (a7 + x_sq .* (a9 + x_sq * a11)))));
    
    res(swap & atan_input >= 0) = pi/2 - res(swap & atan_input >= 0);
    res(swap & atan_input < 0) = -pi/2 - res(swap & atan_input < 0);
    
    mask_q2 = (x < 0) & (y >= 0);
    mask_q3 = (x < 0) & (y < 0);
    res(mask_q2) = pi + res(mask_q2);
    res(mask_q3) = -pi + res(mask_q3);
end

%% asin近似
function res = asin_approx(x)
    res = zeros(size(x));
    mask_high = x >= 1.0;
    mask_low = x <= -1.0;
    mask_normal = ~mask_high & ~mask_low;
    
    res(mask_high) = pi/2;
    res(mask_low) = -pi/2;
    res(mask_normal) = atan2_approx(x(mask_normal), sqrt(1.0 - x(mask_normal).^2));
end

%% 四元数转欧拉角 (ENU)
function att = q2att_ENU(q)
    q0 = q(:,1); q1 = q(:,2); q2 = q(:,3); q3 = q(:,4);
    
    q00 = q0.*q0; q01 = q0.*q1; q02 = q0.*q2; q03 = q0.*q3;
    q11 = q1.*q1; q12 = q1.*q2; q13 = q1.*q3;
    q22 = q2.*q2; q23 = q2.*q3; q33 = q3.*q3;
    
    C11 = q00 + q11 - q22 - q33;
    C13 = 2 * (q13 + q02);
    C32 = 2 * (q23 + q01);
    
    pitch = asin_approx(2 * (q23 + q01));
    
    att = zeros(size(q, 1), 3);
    att(:,2) = pitch;
    
    for i = 1:size(q, 1)
        if C32(i) > 0.997
            att(i,1) = atan2_approx(C13(i), C11(i)) - att(i,3);
            if att(i,1) > pi, att(i,1) = att(i,1) - 2*pi; end
            if att(i,1) < -pi, att(i,1) = att(i,1) + 2*pi; end
        elseif C32(i) < -0.997
            att(i,1) = atan2_approx(C13(i), C11(i)) + att(i,3);
            if att(i,1) > pi, att(i,1) = att(i,1) - 2*pi; end
            if att(i,1) < -pi, att(i,1) = att(i,1) + 2*pi; end
        else
            att(i,1) = -atan2_approx(2 * (q13(i) - q02(i)), q00(i) - q11(i) - q22(i) + q33(i));
            att(i,3) = -atan2_approx(2 * (q12(i) - q03(i)), q00(i) - q11(i) + q22(i) - q33(i));
        end
    end
end

%% 欧拉角转四元数 (ENU)
function q = att2q_ENU(att)
    roll = att(:,1);
    pitch = att(:,2);
    yaw = att(:,3);
    
    sp = sin(pitch/2); cp = cos(pitch/2);
    sr = sin(roll/2);  cr = cos(roll/2);
    sy = sin(yaw/2);   cy = cos(yaw/2);
    
    q = zeros(size(att, 1), 4);
    q(:,1) = cp.*cr.*cy - sp.*sr.*sy;
    q(:,2) = sp.*cr.*cy - cp.*sr.*sy;
    q(:,3) = cp.*sr.*cy + sp.*cr.*sy;
    q(:,4) = cp.*cr.*sy + sp.*sr.*cy;
end
