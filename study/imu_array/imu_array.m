clear;
clc;
close all;
format short

% 分析阵列IMU 每一个IMU单独校准好，还是平均之后当作一个IMU ，然后校准一次好

% 误差因子
error_factor = 0.5;

% 生成随机误差矩阵
C1 = eye(3) + randn(3)*error_factor;
C2 = eye(3) + randn(3)*error_factor;
C3 = eye(3) + randn(3)*error_factor;
C4 = eye(3) + randn(3)*error_factor;


% 每个IMU的bias
B1 = randn(3,1)*error_factor;
B2 = randn(3,1)*error_factor;
B3 = randn(3,1)*error_factor;
B4 = randn(3,1)*error_factor;

% 理论值 标定集
X = [50 0 0; 0 50 0; 0 0 50; 0 0 0; -50 0 0; 0 -50 0; 0 0 -50]';

% 单独每个IMU测量值
IMU1_meas = inv(C1)*(X+B1);
IMU2_meas = inv(C2)*(X+B2);
IMU3_meas = inv(C3)*(X+B3);
IMU4_meas = inv(C4)*(X+B4);



% VIMU(虚拟IMU)的测量值
vimu_meas = (IMU1_meas + IMU2_meas + IMU3_meas + IMU4_meas)/4;

% 陀螺校准, C和B为平均后当成一个IMU校准得到的校准参数
[C, B]= gyr_calibration(vimu_meas', X');

% 测试集
X2 = [11 22 33; -44 66 11; 0 0 0; 1 1 1; 123 131 11]';

IMU1_meas = inv(C1)*(X2+B1);
IMU2_meas = inv(C2)*(X2+B2);
IMU3_meas = inv(C3)*(X2+B3);
IMU4_meas = inv(C4)*(X2+B4);

vimu_meas = (IMU1_meas + IMU2_meas + IMU3_meas + IMU4_meas)/4;

% % 使用单个校准矩阵得到的结果
result_using_single_calibration_matrix = (C*vimu_meas) - B




 