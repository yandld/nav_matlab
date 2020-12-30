%% INS惯导解算
clear;
clc;
close all;

%%
Fs = 100;
N = Fs*10;

gyr = [4 5 6];
acc = [-0.01 0.01 1-0.01];

acc = acc * 9.795;
gyr = deg2rad(gyr);

% 捷联惯导解算
x = zeros(10,1);
x(7:10) = [1 0 0 0];

for i=1:N
    u = [acc gyr]';
    x = ch_nav_equ_local_tan(x, u , 1 / Fs, [0, 0, -9.795]');
    pos(i,:) = x(1:3);
end


ch_plot_pos3d( 'p1', pos, 'title', '3D轨迹', 'legend', ["pos"]');
ch_plot_pos2d( 'p1', pos, 'title', '2D轨迹', 'legend', ["pos"]');

fprintf('纯积分测试: 陀螺bias(rad):%.3f %.3f %.3f\n', gyr(1), gyr(2), gyr(3));
fprintf('纯积分测试: 加计bias(m/s^(2)):%.3f %.3f %.3f\n', acc(1), acc(2), acc(3));

fprintf('解算:%d次 总时间:%.3fs\n', N, N /Fs);
fprintf('最终误差(m): %.3f %.3f %.3f\n', pos(N, 1),  pos(N, 2),  pos(N, 3));

