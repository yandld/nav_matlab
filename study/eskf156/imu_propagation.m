function [X, P, f_n] = imu_propagation(X, P, f_b, Cb2n, Q, dt)
% IMU_PROPAGATION 使用IMU数据进行误差状态预测
%
% 输入:
%   X - 当前误差状态估计 (nx1): 维度由系统设计决定
%   P - 当前误差状态协方差矩阵 (nxn)
%   f_b - 体坐标系下的比力 (3x1)
%   Cb2n - 当前姿态矩阵 (3x3)
%   Q - 系统噪声协方差矩阵 (nxn)
%   dt - 时间步长
%
% 输出:
%   X - 预测后的误差状态估计 (nx1)
%   P - 预测后的误差状态协方差矩阵 (nxn)
%   f_n - 导航系下的比力 (3x1)

% 获取状态维度
n = length(X);

% 计算导航系下的比力
f_n = Cb2n * f_b;

% 初始化误差状态转移矩阵 F
F = zeros(n);

% 基本INS误差模型部分
F(4:6, 1:3) = skew(f_n);
F(7:9, 4:6) = eye(3);

% 添加加速度计偏差影响（假设加速度计偏差状态从索引13开始）
F(4:6, 13:15) = Cb2n;
F(1:3, 10:12) = -Cb2n;

% 离散化状态转移矩阵
Phi = eye(n) + F * dt;

% 误差状态预测
X = Phi * X;

% 误差协方差预测
P = Phi * P * Phi' + Q * dt;

% 确保协方差矩阵保持对称性
 P = (P + P') / 2;

end

function S = skew(v)
% SKEW 创建一个3x3的斜对称矩阵
S = [0    -v(3)  v(2);
    v(3)  0    -v(1);
    -v(2)  v(1)  0];
end
