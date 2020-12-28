%% INS惯导解算
clear;
clc;
close all;

%%
Fs = 100;
N = Fs*10;

gyr = [0 0 0];
acc = [0 0.1 1-0.01];

acc = acc * 9.795;
gyr = deg2rad(gyr);

% 惯导解算
x = zeros(10,1);
x(7:10) = [1 0 0 0];

for i=1:N
    u = [acc gyr]';
    x = ch_nav_equ_local_tan(x, u , 1 / Fs, [0, 0, -9.795]');
    pos(i,:) = x(1:3);
end


ch_plot_pos3d( 'p1', pos, 'title', '3D轨迹', 'legend', ["pos"]');
ch_plot_pos2d( 'p1', pos, 'title', '2D轨迹', 'legend', ["pos"]');

fprintf('计算:%d次 总时间:%.3fs\n', N, N /Fs);
