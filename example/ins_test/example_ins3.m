clear;
clc;
close all;

%% 读取数据 数据来源：超核电子HI226 6轴传感器 www.hipnuc.com

%% 角速度为rad， 加速度为m/s^(2)
load example_ins3_hi226_static_30s.mat

Fs = 100;
N = length(acc);

%% 打印原始数据
ch_plot_imu('time', 1:N, 'acc', acc, 'gyr', gyr);


% 惯导解算, 初始化
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';


for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, -9.8]');
    h_pos(i,:) = p;
    h_eul(i,:) = ch_q2eul(q);
end

figure;
subplot(2,2,1);
ch_plot_pos3d(h_pos);
subplot(2,2,2);
ch_plot_pos2d(h_pos);
subplot(2,2,3);
ch_plot_att(h_eul);


fprintf("共%d数据，用时:%.3fs\n", N, N/Fs);
fprintf("起始位置:%.3f %.3f, 终点位置%.3f %.3f, 相差:%.3fm\n", h_pos(1,1), h_pos(1,2), h_pos(N,1), h_pos(N,2), norm(h_pos(N,:) - h_pos(1,:)));

