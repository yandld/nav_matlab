%% INS惯导解算
clear;
clc;
close all;

%% 
Fs = 100;  %采样频率
N = 1000; %采样次数

dt = 1 / Fs;
gyr = [0.01, 0.02, 0.03]; %单位rad
acc = [0, 0, 9.8]; % 单位m/s^(2)

% 捷联惯导解算
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';

for i=1:N
    [p ,v, q] = ch_nav_equ_local_tan(p, v, q, acc', gyr' , dt, [0, 0, -9.8]');
    h_pos(i,:) = p;
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

fprintf('纯积分测试: 陀螺(rad/s):%.3f %.3f %.3f\n', gyr(1), gyr(2), gyr(3));
fprintf('纯积分测试: 加计(m/s^(2)):%.3f %.3f %.3f\n', acc(1), acc(2), acc(3));

fprintf('解算:%d次 总时间:%.3fs\n', N, N /Fs);
fprintf('最终误差(m): %.3f %.3f %.3f\n', h_pos(end, 1),  h_pos(end, 2),  h_pos(end, 3));


