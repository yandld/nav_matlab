%% example 惯导解算
clear;
clc;
close all;

%%
Fs = 100;

%% GNSS-SIM仿真软件真值(gt= groud true,真实值)
% 使用 https://github.com/Aceinna/gnss-ins-sim 生成仿真数据
 load example_ins2.mat;
 
 %%单位:  m/s^(2), rad/s
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
plot(h_pos(:,1), h_pos(:,2), '.g');
legend("GT", "解算结果");
title("平面位置");


fprintf('总时间:%fs  最终位置差:%f\n', N /Fs,  norm(pos_gt(N, :) - h_pos(N, :)));
