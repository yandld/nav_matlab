%% 加速度计六面校准算法 - 最小二乘法
% 该程序实现了加速度计的六面校准，通过最小二乘法估计校准矩阵和零偏
% 六面校准法是将加速度计的六个面分别朝上放置，收集数据并计算校准参数

clc;
clear;
close all;
format short;

%% 校准数据
% 六个面的加速度计原始读数 [x, y, z], 单位为G, BMI088
input = [
    0.9896   0.0121  -0.0193;  % +X面朝上
   -0.0124   1.0013   0.0024;  % +Y面朝上
   -0.0083  -0.0087   0.9944;  % +Z面朝上
   -1.0094  -0.0084   0.0018;  % -X面朝上
   -0.0068  -0.9974  -0.0198;  % -Y面朝上
   -0.0119   0.0115  -1.0081   % -Z面朝上
];

%% 执行校准计算
[C, B] = acc_calibration(input);

% 显示校准结果
fprintf('校准矩阵:\n');
disp(C);
fprintf('零偏:\n');
disp(B);

%% 应用校准参数计算校准后的数据
output = zeros(size(input));
for i = 1:size(input, 1)
    output(i,:) = (C * input(i,:)' - B)';
end

%% 计算校准前后误差
% 理想值矩阵 - 在标准重力场下六个面的理想读数
ideal_values = [
    1  0  0;   % +X面朝上
    0  1  0;   % +Y面朝上
    0  0  1;   % +Z面朝上
   -1  0  0;   % -X面朝上
    0 -1  0;   % -Y面朝上
    0  0 -1    % -Z面朝上
];

% 计算校准前误差
err_before = input - ideal_values;
residual_before = sum(sqrt(sum(err_before.^2, 2)));

% 计算校准后误差
err_after = output - ideal_values;
residual_after = sum(sqrt(sum(err_after.^2, 2)));

fprintf('校准前误差: %f    校准后误差: %f\n', residual_before, residual_after);
fprintf('误差改善率: %.2f%%\n', (1 - residual_after/residual_before) * 100);


% % 为了完整性，以下是校准函数的实现
% function [C, B] = acc_calibration(input)
% % 加速度计校准函数
% % 输入: input - 6x3矩阵，包含六个面的加速度计读数
% % 输出: C - 3x3校准矩阵
% %       B - 3x1零偏向量
% 
% % 理想值 - 标准重力场下的理想读数
% ideal = [
%     1  0  0;   % +X面朝上
%     0  1  0;   % +Y面朝上
%     0  0  1;   % +Z面朝上
%     -1  0  0;   % -X面朝上
%     0 -1  0;   % -Y面朝上
%     0  0 -1    % -Z面朝上
%     ];
% 
% % 构建方程组 A*x = b
% A = zeros(18, 12);
% b = zeros(18, 1);
% 
% for i = 1:6
%     A(3*i-2, 1:3) = input(i, :);
%     A(3*i-2, 10) = 1;
%     b(3*i-2) = ideal(i, 1);
% 
%     A(3*i-1, 4:6) = input(i, :);
%     A(3*i-1, 11) = 1;
%     b(3*i-1) = ideal(i, 2);
% 
%     A(3*i, 7:9) = input(i, :);
%     A(3*i, 12) = 1;
%     b(3*i) = ideal(i, 3);
% end
% 
% % 使用最小二乘法求解
% x = A \ b;
% 
% % 重构校准矩阵和零偏
% C = [
%     x(1:3)';
%     x(4:6)';
%     x(7:9)'
%     ];
% 
% B = -[x(10); x(11); x(12)];
% end
