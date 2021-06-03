%% example 惯导解算
clear;
clc;
close all;

%% 
Fs = 100;

%% GNSS-SIM仿真软件真值(gt= groud true,真实值)
% 使用 https://github.com/Aceinna/gnss-ins-sim 生成仿真数据

 %%单位:  ACC: m/s^(2),  GYR: rad/s
 load example_ins2.mat;
 

N = length(acc);

% 惯导解算, 初始化
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';


for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, 9.8]');
    h_pos(i,:) = p;
    h_att(i,:) = rad2deg(ch_q2eul(q))';
    h_vel(i,:) = v;
    h_eul(i,:) = ch_q2eul(q);
end

figure;
subplot(2,2,1);
ch_plot_pos3d(h_pos);
subplot(2,2,2);
ch_plot_pos2d(h_pos);
subplot(2,2,3);
ch_plot_att(h_eul);

figure;
plot(pos_gt(:,1), pos_gt(:,2), '.r');
hold on;
plot(h_pos(:,1), h_pos(:,2), '.g');
legend("Groud True", "解算结果");
title("平面位置");


fprintf('总时间:%fs  最终位置差:%f\n', N /Fs,  norm(pos_gt(N, :) - h_pos(N, :)));
