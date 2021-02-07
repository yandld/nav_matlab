clear;
clc
close all;

format long

%% Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2nd Edition, by Paul D. Groves EXAMPLE 5.3


lat = deg2rad(45); %当地纬度
eul_true = deg2rad([2 -3 30]); %仿真真实欧拉角
roll = eul_true(1);
pitch = eul_true(2);
yaw = eul_true(3);
w_e2i = 7.29e-5; %地球自转角速度


acc_error = [0.011 -0.007 0.009]'; %加速度测量误差
gyr_error = [1.20e-6  -1.8e-6  2.5e-6]'; % 陀螺测量误差， 单位为rad

% Accelerometer measurements
sf = [sin(pitch) -cos(pitch)*sin(roll) -cos(pitch)*cos(roll)]' * 9.8; %得到比利在B系下投影
sf_m = sf + acc_error;
fprintf("测量得到的比力: %f %f %f\n", sf_m(1), sf_m(2), sf_m(3));

Cn2b = ch_eul2m(eul_true);

w_b2i_b = Cn2b * [cos(lat)*w_e2i 0 -sin(lat)*w_e2i]';

w_m_b2i_b = w_b2i_b + gyr_error; %测量到的角速度
fprintf("测量得到的角速度: %.8f %.8f %.8f\n", w_m_b2i_b(1), w_m_b2i_b(2), w_m_b2i_b(3));

%% leveling
a1 = sf_m(1);
a2 = sqrt( sf_m(2)^(2) + sf_m(3)^(2) );
pitch_m = atan2(a1, a2);
roll_m = atan2(-sf_m(2), -sf_m(3));
fprintf("测量得到的俯仰横滚:  roll:%f deg pitch%f deg\n", rad2deg(roll_m), rad2deg(pitch_m) );


% gyrocompassing
sin_yaw = -w_m_b2i_b(2)*cos(roll_m) + w_m_b2i_b(3)*sin(roll_m);
cos_yaw = w_m_b2i_b(1) * cos(pitch_m)  + w_m_b2i_b(2) * sin(roll_m)*sin(pitch_m) + w_m_b2i_b(3) * cos(roll_m)*sin(pitch_m);

yaw = atan2(sin_yaw, cos_yaw);

fprintf("yaw角:%f\n", rad2deg(yaw));


