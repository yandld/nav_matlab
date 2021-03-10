clear;
clc;
close all;
format

%% 读取数据
data = csvread("UranusData.csv", 1, 1);

Fs = 100;

accBias =  mean(data(1:200,2:4));
accBias(:,3) = accBias(:,3) - 1;

% 减去零偏
accReading = data(:,2:4);
accReading =  accReading -  accBias;
accReading = accReading*9.795;

gyrReading = data(:,5:7);
gyrReading = deg2rad(gyrReading);

N = length(accReading);


x = zeros(10,1);
x(7:10) = [1 0 0 0];


for i=1:N
    u = [accReading(i,:) gyrReading(i,:)]';
    x = ch_nav_equ_local_tan(x, u , 1 / Fs, [0, 0, -9.795]');
    
    gyr = u(4:6);
    if(norm(gyr) < deg2rad(1))
         x(4:6) = 0;
    end

    pos_matlab(i,:) = x(1:3);
    att_matlab(i,:) = rad2deg(ch_q2eul(x(7:10)));
    vel_matlab(i,:) = x(4:6);
end

%% 姿态
ch_imu_data_plot('time', 1:N, 'acc', accReading, 'gyr', gyrReading,  'eul', att_matlab, 'subplot', true);

figure;
plot(vel_matlab);
title('速度');


%3D位置plot
figure;
plot3(pos_matlab(1,1), pos_matlab(1,2), pos_matlab(1,3), '-ks');
hold on;
plot3(pos_matlab(:,1), pos_matlab(:,2), pos_matlab(:,3), '.b');
axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
title('3D位置');
legend('起始', '3D');


figure;
plot(pos_matlab(:,1), pos_matlab(:,2), '.b');
hold on;
plot(pos_matlab(1,1), pos_matlab(1,2), '-ks');
axis equal
title('2D位置');
xlabel('X(m)');  ylabel('Y(m)'); 

    
fprintf("共%d数据，用时:%.3fs\n", N, N/Fs);
fprintf("起始位置:%.3f %.3f, 终点位置%.3f %.3f, 相差:%.3fm\n", pos_matlab(1,1), pos_matlab(1,2), pos_matlab(N,1), pos_matlab(N,2), norm(pos_matlab(N,:) - pos_matlab(1,:)));

