%% 基于卡尔曼滤波的磁力计校准算法 - 修正版本
% 使用正确的四元数到DCM转换
% by WF
clear; clc; close all;
%% 设置零偏真值
mag_bias_truth = [0 0 0];
%% ========== 1. 数据读取和预处理 ==========
% 修改这里的文件名
data_file = 'mag_test.csv';  % <-- 改成你的数据文件名
dt = 0.01; % 采样dt
fprintf('正在读取数据文件: %s\n', data_file);
data = readtable(data_file);
%data = data(1:250,:);
% 提取需要的数据列
quat_w = data.quat_w;
quat_x = data.quat_x; 
quat_y = data.quat_y;
quat_z = data.quat_z;
mag_x = data.mag_x;
mag_y = data.mag_y;
mag_z = data.mag_z;
gyr_x = data.gyr_x;
gyr_y = data.gyr_y;
gyr_z = data.gyr_z;

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
gyr_x = gyr_x(valid_idx);
gyr_y = gyr_y(valid_idx);
gyr_z = gyr_z(valid_idx);

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

%% ========== 5. 方法1: 基于四元数的卡尔曼滤波器 ==========
% 状态向量: x = [Bn_x, Bn_y, Bn_z, bias_x, bias_y, bias_z]'
n_states = 6;

% 初始状态估计（使用前几个数据点的平均值）
n_init = min(50, N);
mag_init_mean = [mean(mag_x(1:n_init)); mean(mag_y(1:n_init)); mean(mag_z(1:n_init))];

% 初始状态: 假设零偏为0，地磁矢量为观测均值
x_est = [mag_init_mean; 0; 0; 0];

% 初始协方差矩阵（较大的不确定性）
P_est = diag([1e3, 1e3, 1e3, 1e2, 1e2, 1e2]); % [nT^2]

% 过程噪声协方差矩阵 - 可以调试这些参数！
Q_Bn = 0.001*1e0;    % 地磁矢量过程噪声 [nT^2]
Q_bias = 0.001*1e-1;  % 零偏过程噪声 [nT^2]
Q = diag([Q_Bn, Q_Bn, Q_Bn, Q_bias, Q_bias, Q_bias]);

% 观测噪声协方差矩阵 - 根据磁力计精度调整
R_mag = 2;   % 磁力计观测噪声 [nT^2]
R = diag([R_mag, R_mag, R_mag]);

% 状态转移矩阵（恒等矩阵）
F = eye(n_states);

% 存储数组初始化
bias_history_method1 = zeros(N, 3);

% 卡尔曼滤波主循环
for k = 1:N
  %% 预测步骤
  x_pred = F * x_est;
  P_pred = F * P_est * F' + Q;
  
  %% 构造观测矩阵H (使用正确的DCM转换)
  q_current = [quat_w(k), quat_x(k), quat_y(k), quat_z(k)];
  
  % 使用正确的四元数到DCM转换
  Cb2n = quat2dcm_correct(q_current);
  Cn2b = Cb2n';
  % 观测矩阵: z = Cn2b * Bn + bias
  H = [Cn2b, eye(3)];
  
  %% 更新步骤
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
  
  % 保存零偏结果
  bias_history_method1(k, :) = x_est(4:6)';
end

% 方法1结果
bias_est_method1 = x_est(4:6);

%% ========== 6. 方法2: KF结合陀螺仪数据得到硬磁估计 ==========
% 使用全部数据集
cal_gyr = [gyr_x, gyr_y, gyr_z];
cal_mag = [mag_x, mag_y, mag_z];

X = zeros(6,1);
X(1:3) = cal_mag(1,:)';

Q_method2 = blkdiag(eye(3)*0.1^(2), eye(3)*0.1^(2));
R_method2 = eye(3) * 1^(2);
P_method2 = eye(6)*100;

bias_history_method2 = zeros(N, 3);

for i=1:N
    gyr = deg2rad(cal_gyr(i,:)');
    y = cal_mag(i,:)';
    sk = ch_askew(gyr);
    F_method2 = [-sk, sk; zeros(3,6)];
    F_method2 = eye(6) + F_method2*dt;
    
    % predict
    X = F_method2*X;
    P_method2 = F_method2*P_method2*F_method2' + Q_method2;
    
    % update
    H_method2 = [eye(3) zeros(3,3)];
    
    K_method2 = P_method2*H_method2'*(H_method2*P_method2*H_method2' + R_method2)^(-1);
    X = X + K_method2*(y - H_method2*X);
    P_method2 = (eye(6) - K_method2*H_method2)*P_method2;
    
    bias_history_method2(i, :) = X(4:6)';
end

bias_est_method2 = X(4:6);

%% ========== 7. 方法3: RLS估计硬磁 ==========
X_rls = zeros(4,1);
P_rls = eye(4)*10;
lamda = 1.0;

bias_history_method3 = zeros(N, 3);

for i=1:N
    y = cal_mag(i,:)';
    H_rls = [y; 1]';
    K_rls = P_rls*H_rls' / (H_rls*P_rls*H_rls' + lamda);
    P_rls = (eye(4) - K_rls*H_rls)*P_rls / lamda;
    X_rls = X_rls + K_rls*(y'*y - H_rls*X_rls);

    bias_history_method3(i, :) = [X_rls(1)/2, X_rls(2)/2, X_rls(3)/2];
end

bias_est_method3 = [X_rls(1)/2, X_rls(2)/2, X_rls(3)/2];

%% ========== 8. 方法4: 椭球拟合法 ==========
[A_ellipsoid, bias_est_method4, expmfs] = magcal(cal_mag, 'auto');

%% ========== 9. 结果输出 ==========
fprintf('\n');
fprintf('============================================================\n');
fprintf('磁力计校准方法对比总结\n');
fprintf('============================================================\n');

fprintf('真值:                    bias=[%.2f, %.2f, %.2f]\n', mag_bias_truth);
fprintf('方法1 - 基于四元数的KF: bias=[%.2f, %.2f, %.2f]\n', bias_est_method1);
fprintf('方法2 - KF结合陀螺仪:    bias=[%.2f, %.2f, %.2f]\n', bias_est_method2);
fprintf('方法3 - RLS方法:         bias=[%.2f, %.2f, %.2f]\n', bias_est_method3);
fprintf('方法4 - 椭球拟合:        bias=[%.2f, %.2f, %.2f]\n', bias_est_method4);

%% ========== 10. 零偏实时估计对比绘图 ==========
figure('Position', [100, 100, 1200, 800]);

% X轴零偏对比
subplot(2, 2, 1);
plot(1:N, bias_history_method1(:, 1), 'r-', 'LineWidth', 2, 'DisplayName', '方法1-四元数KF');
hold on;
plot(1:N, bias_history_method2(:, 1), 'g-', 'LineWidth', 2, 'DisplayName', '方法2-KF+陀螺仪');
plot(1:N, bias_history_method3(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '方法3-RLS');
yline(bias_est_method4(1), 'k--', 'LineWidth', 2, 'DisplayName', '方法4-椭球拟合');
yline(mag_bias_truth(1), 'm--', 'LineWidth', 2, 'DisplayName', '真值');
xlabel('数据点');
ylabel('X轴零偏 [nT]');
title('X轴零偏实时估计对比');
legend('Location', 'best');
grid on;

% Y轴零偏对比
subplot(2, 2, 2);
plot(1:N, bias_history_method1(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', '方法1-四元数KF');
hold on;
plot(1:N, bias_history_method2(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', '方法2-KF+陀螺仪');
plot(1:N, bias_history_method3(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '方法3-RLS');
yline(bias_est_method4(2), 'k--', 'LineWidth', 2, 'DisplayName', '方法4-椭球拟合');
yline(mag_bias_truth(2), 'm--', 'LineWidth', 2, 'DisplayName', '真值');
xlabel('数据点');
ylabel('Y轴零偏 [nT]');
title('Y轴零偏实时估计对比');
legend('Location', 'best');
grid on;

% Z轴零偏对比
subplot(2, 2, 3);
plot(1:N, bias_history_method1(:, 3), 'r-', 'LineWidth', 2, 'DisplayName', '方法1-四元数KF');
hold on;
plot(1:N, bias_history_method2(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', '方法2-KF+陀螺仪');
plot(1:N, bias_history_method3(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', '方法3-RLS');
yline(bias_est_method4(3), 'k--', 'LineWidth', 2, 'DisplayName', '方法4-椭球拟合');
yline(mag_bias_truth(3), 'm--', 'LineWidth', 2, 'DisplayName', '真值');
xlabel('数据点');
ylabel('Z轴零偏 [nT]');
title('Z轴零偏实时估计对比');
legend('Location', 'best');
grid on;

% 零偏模长对比
subplot(2, 2, 4);
bias_norm_method1 = sqrt(sum(bias_history_method1.^2, 2));
bias_norm_method2 = sqrt(sum(bias_history_method2.^2, 2));
bias_norm_method3 = sqrt(sum(bias_history_method3.^2, 2));
bias_norm_method4 = norm(bias_est_method4);
bias_norm_truth = norm(mag_bias_truth);

plot(1:N, bias_norm_method1, 'r-', 'LineWidth', 2, 'DisplayName', '方法1-四元数KF');
hold on;
plot(1:N, bias_norm_method2, 'g-', 'LineWidth', 2, 'DisplayName', '方法2-KF+陀螺仪');
plot(1:N, bias_norm_method3, 'b-', 'LineWidth', 2, 'DisplayName', '方法3-RLS');
yline(bias_norm_method4, 'k--', 'LineWidth', 2, 'DisplayName', '方法4-椭球拟合');
yline(bias_norm_truth, 'm--', 'LineWidth', 2, 'DisplayName', '真值');
xlabel('数据点');
ylabel('零偏模长 [nT]');
title('零偏模长实时估计对比');
legend('Location', 'best');
grid on;

function m = ch_askew(v) 
% 生成反对称矩阵
    m = [ 0,     -v(3),   v(2); 
          v(3),   0,     -v(1); 
         -v(2),   v(1),   0     ];
end

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
