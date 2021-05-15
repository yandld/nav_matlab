%% example 惯导解算
clear;
clc;
close all;

%%
Fs = 100;


gravity = 9.80235145615448;


%% GNSS-SIM仿真软件真值(gt= groud true,真实值)
pos_gt = csvread('generated_data/ref_pos.csv', 1, 0);
pos_gt = pos_gt - pos_gt(1,:);
att_gt =  csvread('generated_data/ref_att_euler.csv', 1, 0);
vel_gt =  csvread('generated_data/ref_vel.csv', 1, 0);

%%单位:  m/s^(2), deg/s
 acc = csvread('generated_data/accel-0.csv', 1, 0);
 gyr = csvread('generated_data/gyro-0.csv', 1, 0);

gyr = deg2rad(gyr);
N = length(acc);

% 惯导解算, 初始化
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';

gravity = 9.79444435359668;

for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, gravity]');
    pos(i,:) = p;
    att(i,:) = rad2deg(ch_q2eul(q))';
    vel(i,:) = v;
end

figure;
subplot(2,1,1);
plot(acc);
title("ACC");
legend("X", "Y", "Z");

subplot(2,1,2);
plot(gyr);
title("GYR");
legend("X", "Y", "Z");

figure;
plot(pos_gt(:,1), pos_gt(:,2), '.r');
hold on;
plot(pos(:,1), pos(:,2), '.g');
legend("GT", "解算结果");
title("平面位置");


fprintf('总时间:%fs  最终位置差:%f\n', N /Fs,  norm(pos_gt(N, :) - pos(N, :)));
