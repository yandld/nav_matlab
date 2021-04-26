clear;
clc;
close all;
format

%% 读取数据
load example_ins4.mat

% %% 读取数据
% load hi226_static_30s.mat

Fs = 400; 
N = length(acc);

%% 打印原始数据
ch_plot_imu('time', 1:N, 'acc', acc, 'gyr', gyr);

%% 角速度变为rad， 加速度变为m/s^(2)
gyr = deg2rad(gyr);
acc = acc*9.795;

% 惯导解算, 初始化
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';


for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, -9.795]');
    pos(i,:) = p;
end

%3D位置plot
figure;
plot3(pos(1,1), pos(1,2), pos(1,3), '-ks');
hold on;
plot3(pos(:,1), pos(:,2), pos(:,3), '.b');
axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
title('3D位置');
legend('起始', '3D');


figure;
plot(pos(:,1), pos(:,2), '.b');
hold on;
plot(pos(1,1), pos(1,2), '-ks');
axis equal
title('2D位置');
xlabel('X(m)');  ylabel('Y(m)'); 

    
fprintf("共%d数据，用时:%.3fs\n", N, N/Fs);
fprintf("起始位置:%.3f %.3f, 终点位置%.3f %.3f, 相差:%.3fm\n", pos(1,1), pos(1,2), pos(N,1), pos(N,2), norm(pos(N,:) - pos(1,:)));

