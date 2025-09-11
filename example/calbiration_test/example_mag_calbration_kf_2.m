%% 基于卡尔曼滤波的磁力计校准算法 - 修正版本
% 使用正确的四元数到DCM转换

clear; clc; close all;

%% ========== 1. 数据读取和预处理 ==========
% 修改这里的文件名
data_file = 'mag_test.csv';  % <-- 改成你的数据文件名

fprintf('正在读取数据文件: %s\n', data_file);
data = readtable(data_file);
%data = data(1:5500,:);
% 提取需要的数据列
quat_w = data.quat_w;
quat_x = data.quat_x; 
quat_y = data.quat_y;
quat_z = data.quat_z;
mag_x = data.mag_x;
mag_y = data.mag_y;
mag_z = data.mag_z;

N = length(quat_w);
fprintf('数据点数: %d\n', N);

%% ========== 2. 数据质量检查 ==========
% 检查四元数归一化
quat_norm = sqrt(quat_w.^2 + quat_x.^2 + quat_y.^2 + quat_z.^2);
quat_norm_error = abs(quat_norm - 1);
bad_quat_idx = quat_norm_error > 0.1;

fprintf('四元数归一化检查: %.1f%% 的数据正常\n', (1-sum(bad_quat_idx)/N)*100);

if sum(bad_quat_idx) > 0
  fprintf('警告: 发现 %d 个异常四元数，将进行归一化处理\n', sum(bad_quat_idx));
  quat_w(bad_quat_idx) = quat_w(bad_quat_idx) ./ quat_norm(bad_quat_idx);
  quat_x(bad_quat_idx) = quat_x(bad_quat_idx) ./ quat_norm(bad_quat_idx);
  quat_y(bad_quat_idx) = quat_y(bad_quat_idx) ./ quat_norm(bad_quat_idx);
  quat_z(bad_quat_idx) = quat_z(bad_quat_idx) ./ quat_norm(bad_quat_idx);
end

% 检查磁力计数据异常值
mag_norm = sqrt(mag_x.^2 + mag_y.^2 + mag_z.^2);
mag_median = median(mag_norm);
mag_std = std(mag_norm);
bad_mag_idx = abs(mag_norm - mag_median) > 3 * mag_std;

fprintf('磁力计数据检查: %.1f%% 的数据正常\n', (1-sum(bad_mag_idx)/N)*100);
fprintf('磁力计模长: 均值=%.1f nT, 标准差=%.1f nT\n', mag_median, mag_std);

if sum(bad_mag_idx) > 0
  fprintf('警告: 发现 %d 个磁力计异常值\n', sum(bad_mag_idx));
end

% 移除异常数据点
valid_idx = ~bad_quat_idx & ~bad_mag_idx;
quat_w = quat_w(valid_idx);
quat_x = quat_x(valid_idx);
quat_y = quat_y(valid_idx);
quat_z = quat_z(valid_idx);
mag_x = mag_x(valid_idx);
mag_y = mag_y(valid_idx);
mag_z = mag_z(valid_idx);

N = length(quat_w);
fprintf('有效数据点数: %d\n', N);



%% ========== 4. 验证转换正确性 ==========
fprintf('\n=== 验证DCM转换 ===\n');

% 测试第一个四元数
q_test = [quat_w(1), quat_x(1), quat_y(1), quat_z(1)];

% 使用修正后的转换
C_correct = quat2dcm_correct(q_test);

% 使用MATLAB内置函数对比 (如果可用)
try
  % MATLAB的四元数顺序是 [w, x, y, z]
  C_matlab = quat2rotm([quat_w(1), quat_x(1), quat_y(1), quat_z(1)]);
  diff_matlab = max(abs(C_correct(:) - C_matlab(:)));
  fprintf('与MATLAB内置函数差异: %.2e\n', diff_matlab);
catch
  fprintf('MATLAB内置四元数函数不可用，跳过对比\n');
end

% 验证DCM的正交性
det_C = det(C_correct);
orthogonality_error = max(abs(C_correct * C_correct' - eye(3)), [], 'all');

fprintf('DCM行列式: %.6f (应该为1)\n', det_C);
fprintf('正交性误差: %.2e (应该接近0)\n', orthogonality_error);

if abs(det_C - 1) < 1e-10 && orthogonality_error < 1e-10
  fprintf('✓ DCM转换验证通过\n');
else
  fprintf('✗ DCM转换验证失败\n');
end

%% ========== 5. 卡尔曼滤波器初始化 ==========
% 状态向量: x = [Bn_x, Bn_y, Bn_z, bias_x, bias_y, bias_z]'
n_states = 6;

% 初始状态估计（使用前几个数据点的平均值）
n_init = min(50, N);
mag_init_mean = [mean(mag_x(1:n_init)); mean(mag_y(1:n_init)); mean(mag_z(1:n_init))];

% 初始状态: 假设零偏为0，地磁矢量为观测均值
x_est = [mag_init_mean; 0; 0; 0];
fprintf('\n初始状态估计:\n');
fprintf('  Bn_init = [%.1f, %.1f, %.1f] nT\n', x_est(1), x_est(2), x_est(3));
fprintf('  bias_init = [%.1f, %.1f, %.1f] nT\n', x_est(4), x_est(5), x_est(6));

% 初始协方差矩阵（较大的不确定性）
P_est = diag([1e3, 1e3, 1e3, 1e2, 1e2, 1e2]); % [nT^2]

% 过程噪声协方差矩阵 - 可以调试这些参数！
Q_Bn = 0.0000*1e0;    % 地磁矢量过程噪声 [nT^2]
Q_bias = 0.0000*1e-1;  % 零偏过程噪声 [nT^2]
Q = diag([Q_Bn, Q_Bn, Q_Bn, Q_bias, Q_bias, Q_bias]);

% 观测噪声协方差矩阵 - 根据磁力计精度调整
R_mag = 2;   % 磁力计观测噪声 [nT^2]
R = diag([R_mag, R_mag, R_mag]);

% 状态转移矩阵（恒等矩阵）
F = eye(n_states);

fprintf('滤波器参数:\n');
fprintf('  过程噪声 Q_Bn = %.0e, Q_bias = %.0e\n', Q_Bn, Q_bias);
fprintf('  观测噪声 R_mag = %.0e\n', R_mag);

%% ========== 6. 存储数组初始化 ==========
x_history = zeros(n_states, N);
P_trace = zeros(N, 1);
residuals = zeros(N, 3);
K_gain_norm = zeros(N, 1);

%% ========== 7. 卡尔曼滤波主循环 ==========
fprintf('\n开始卡尔曼滤波处理...\n');
tic; % 计时开始

for k = 1:N
  if mod(k, 1000) == 0
      fprintf('处理进度: %d/%d (%.1f%%), 用时: %.2fs\n', k, N, k/N*100, toc);
  end
  
  %% 7.1 预测步骤
  x_pred = F * x_est;
  P_pred = F * P_est * F' + Q;
  
  %% 7.2 构造观测矩阵H (使用正确的DCM转换)
  q_current = [quat_w(k), quat_x(k), quat_y(k), quat_z(k)];
  
  % 使用正确的四元数到DCM转换
  Cb2n = quat2dcm_correct(q_current);
  Cn2b = Cb2n';
  % 观测矩阵: z = Cn2b * Bn + bias
  H = [Cn2b, eye(3)];
  
  %% 7.3 更新步骤
  z_obs = [mag_x(k); mag_y(k); mag_z(k)];
  z_pred = H * x_pred;
  
  % 新息（残差）
  innovation = z_obs - z_pred;
  
  % 新息协方差
  S = H * P_pred * H' + R;
  
  % 卡尔曼增益
  K = P_pred * H' / S;
  
  % 状态更新
  x_est = x_pred + K * innovation;
  
  % 协方差更新（Joseph形式，数值稳定）
  I_KH = eye(n_states) - K * H;
  P_est = I_KH * P_pred * I_KH' + K * R * K';
  
  % 保存结果
  x_history(:, k) = x_est;
  P_trace(k) = trace(P_est);
  residuals(k, :) = innovation';
  K_gain_norm(k) = norm(K, 'fro');
end

total_time = toc;
fprintf('滤波完成! 总用时: %.2fs\n', total_time);

%% ========== 8. 最终结果 ==========
Bn_est = x_est(1:3);
bias_est = x_est(4:6);

fprintf('\n=== 磁力计校准结果 ===\n');
fprintf('估计的导航系地磁矢量 [nT]:\n');
fprintf('  Bn_x = %.2f\n', Bn_est(1));
fprintf('  Bn_y = %.2f\n', Bn_est(2)); 
fprintf('  Bn_z = %.2f\n', Bn_est(3));
fprintf('  |Bn| = %.2f\n', norm(Bn_est));

fprintf('\n估计的磁力计零偏 [nT]:\n');
fprintf('  bias_x = %.2f\n', bias_est(1));
fprintf('  bias_y = %.2f\n', bias_est(2));
fprintf('  bias_z = %.2f\n', bias_est(3));
fprintf('  |bias| = %.2f\n', norm(bias_est));

%% ========== 9. 残差统计 ==========
residual_norm = sqrt(sum(residuals.^2, 2));
residual_mean = mean(residual_norm);
residual_std = std(residual_norm);
residual_rms = sqrt(mean(residual_norm.^2));

fprintf('\n=== 残差统计 ===\n');
fprintf('  均值: %.2f nT\n', residual_mean);
fprintf('  标准差: %.2f nT\n', residual_std);
fprintf('  RMS: %.2f nT\n', residual_rms);
fprintf('  最大值: %.2f nT\n', max(residual_norm));

%% ========== 10. 结果可视化 ==========
fprintf('\n正在生成可视化结果...\n');

figure('Position', [100, 100, 1400, 1000]);

% 子图1: 地磁矢量估计收敛
subplot(3, 3, 1);
plot(1:N, x_history(1, :), 'r-', 'LineWidth', 1.5); hold on;
plot(1:N, x_history(2, :), 'g-', 'LineWidth', 1.5);
plot(1:N, x_history(3, :), 'b-', 'LineWidth', 1.5);
xlabel('数据点'); ylabel('地磁矢量 [nT]');
title('地磁矢量估计收敛');
legend('Bn_x', 'Bn_y', 'Bn_z', 'Location', 'best');
grid on;

% 子图2: 零偏估计收敛
subplot(3, 3, 2);
plot(1:N, x_history(4, :), 'r-', 'LineWidth', 1.5); hold on;
plot(1:N, x_history(5, :), 'g-', 'LineWidth', 1.5);
plot(1:N, x_history(6, :), 'b-', 'LineWidth', 1.5);
xlabel('数据点'); ylabel('零偏 [nT]');
title('零偏估计收敛');
legend('bias_x', 'bias_y', 'bias_z', 'Location', 'best');
grid on;

% 子图3: 协方差迹
subplot(3, 3, 3);
semilogy(1:N, P_trace, 'b-', 'LineWidth', 1.5);
xlabel('数据点'); ylabel('协方差矩阵迹');
title('估计不确定性变化');
grid on;

% 子图4: 基于地磁矢量估计的航向角变化 - 修改后的版本
subplot(3, 3, 4);

% 计算航向角（基于估计的地磁矢量在水平面的投影）
heading_angles = zeros(N, 1);
for i = 1:N
    Bn_x_est = x_history(1, i);  % 东向地磁分量
    Bn_y_est = x_history(2, i);  % 北向地磁分量
    
    % 计算航向角 (从北向顺时针，弧度)
    heading_angles(i) = atan2(Bn_x_est, Bn_y_est);
end

% 转换为度数
heading_degrees = rad2deg(heading_angles);

% 绘制航向角变化
plot(1:N, heading_degrees, 'k-', 'LineWidth', 2); hold on;

% 添加参考线
yline(0, 'b--', 'Alpha', 0.5, 'LineWidth', 1);
yline(90, 'g--', 'Alpha', 0.5, 'LineWidth', 1);
yline(-90, 'g--', 'Alpha', 0.5, 'LineWidth', 1);
yline(180, 'r--', 'Alpha', 0.5, 'LineWidth', 1);
yline(-180, 'r--', 'Alpha', 0.5, 'LineWidth', 1);

xlabel('数据点'); 
ylabel('航向角 [度]');
title('基于地磁矢量的航向角估计');
grid on; 
xlim([1, N]);
ylim([-180, 180]);

% 添加最终航向值显示
text(0.02, 0.95, sprintf('最终航向: %.1f°', heading_degrees(end)), ...
     'Units', 'normalized', 'FontSize', 10, 'FontWeight', 'bold', ...
     'BackgroundColor', 'white', 'EdgeColor', 'black');

% 子图5: 校准前后对比
subplot(3, 3, 5);
mag_raw = [mag_x, mag_y, mag_z];
mag_calibrated = mag_raw - repmat(bias_est', N, 1);
scatter3(mag_raw(:,1), mag_raw(:,2), mag_raw(:,3), 10, 'r.', 'DisplayName', '校准前');
hold on;
scatter3(mag_calibrated(:,1), mag_calibrated(:,2), mag_calibrated(:,3), 10, 'b.', 'DisplayName', '校准后');
xlabel('X [nT]'); ylabel('Y [nT]'); zlabel('Z [nT]');
title('磁力计数据校准对比');
legend('Location', 'best');
axis equal; grid on;

% 子图6: 残差时间序列
subplot(3, 3, 6);
plot(1:N, residuals(:,1), 'r-', 'LineWidth', 1); hold on;
plot(1:N, residuals(:,2), 'g-', 'LineWidth', 1);
plot(1:N, residuals(:,3), 'b-', 'LineWidth', 1);
xlabel('数据点'); ylabel('残差 [nT]');
title('观测残差');
legend('X', 'Y', 'Z', 'Location', 'best');
grid on;

% 子图7: 残差分布直方图
subplot(3, 3, 7);
histogram(residual_norm, 50, 'Normalization', 'probability');
xlabel('残差模长 [nT]'); ylabel('概率密度');
title('残差分布');
grid on;

% 子图8: 卡尔曼增益变化
subplot(3, 3, 8);
semilogy(1:N, K_gain_norm, 'k-', 'LineWidth', 1.5);
xlabel('数据点'); ylabel('卡尔曼增益范数');
title('卡尔曼增益变化');
grid on;

% 子图9: 磁力计模长对比
subplot(3, 3, 9);
mag_raw_norm = sqrt(sum(mag_raw.^2, 2));
mag_cal_norm = sqrt(sum(mag_calibrated.^2, 2));
plot(1:N, mag_raw_norm, 'r-', 'LineWidth', 1, 'DisplayName', '校准前'); hold on;
plot(1:N, mag_cal_norm, 'b-', 'LineWidth', 1, 'DisplayName', '校准后');
yline(norm(Bn_est), 'k--', 'LineWidth', 2, 'DisplayName', '理论值');
xlabel('数据点'); ylabel('磁力计模长 [nT]');
title('磁力计模长对比');
legend('Location', 'best');
grid on;

fprintf('可视化完成!\n');

%% ========== 11. 调试信息输出 ==========
fprintf('\n=== 调试信息 ===\n');
fprintf('使用的DCM转换方式: 正确的四元数到DCM转换 (基于C代码)\n');
fprintf('工作空间中的主要变量:\n');
fprintf('  x_history: %dx%d 状态历史\n', size(x_history));
fprintf('  residuals: %dx%d 残差历史\n', size(residuals));
fprintf('  P_trace: %dx1 协方差迹\n', length(P_trace));
fprintf('  Bn_est: 最终地磁矢量估计\n');
fprintf('  bias_est: 最终零偏估计\n');
fprintf('  mag_calibrated: %dx3 校准后数据\n', size(mag_calibrated));

fprintf('\n你可以在命令窗口中查看任何变量，例如:\n');
fprintf('  >> std(residuals)                  %% 查看各轴残差标准差\n');
fprintf('  >> norm(Bn_est)                    %% 查看地磁矢量模长\n');
fprintf('  >> plot(mag_cal_norm)              %% 查看校准后模长变化\n');

%% 完成
fprintf('\n=== 处理完成 ===\n');
fprintf('总处理时间: %.2f秒\n', total_time);
fprintf('平均处理速度: %.0f Hz\n', N/total_time);

% 嵌套函数定义
function dcm = quat2dcm_correct(q)
  % q = [qw, qx, qy, qz] - 四元数 [w, x, y, z]
  qw = q(1); qx = q(2); qy = q(3); qz = q(4);
  
  % 按照C代码的实现
  q00 = qw * qw;
  q11 = qx * qx;
  q22 = qy * qy;
  q33 = qz * qz;
  
  q01 = qw * qx;
  q02 = qw * qy;
  q03 = qw * qz;
  q12 = qx * qy;
  q13 = qx * qz;
  q23 = qy * qz;
  
  % 构造DCM矩阵 (body to navigation, bCn)
  dcm = zeros(3,3);
  
  % First row
  dcm(1,1) = q00 + q11 - q22 - q33;
  dcm(1,2) = 2.0 * (q12 - q03);
  dcm(1,3) = 2.0 * (q13 + q02);
  
  % Second row
  dcm(2,1) = 2.0 * (q12 + q03);
  dcm(2,2) = q00 - q11 + q22 - q33;
  dcm(2,3) = 2.0 * (q23 - q01);
  
  % Third row
  dcm(3,1) = 2.0 * (q13 - q02);
  dcm(3,2) = 2.0 * (q23 + q01);
  dcm(3,3) = q00 - q11 - q22 + q33;
end

