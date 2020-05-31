%% Start of script
close all;
clear;
clc;

%% Import and plot sensor data
load('imu_dataset.mat');
%%  plot sensor data 
imu = dataset.imu;

imu_data_plot('acc', imu.acc', 'gyr', imu.gyr', 'mag', imu.mag', 'time', imu.time');
dt = mean(diff(imu.time)); % sample time 
n = length(imu.time);
%% Process sensor data through algorithm

q1 = zeros(4,n);

q1(:,1) = [1 0 0 0]';

%% intgate
for i = 2:n
	q1(:,i) = ch_mahony.imu(q1(:,i-1), imu.gyr(:,i), imu.acc(:,i), dt, 1);
	%q2(i, :) = fourati.imu(q2(i-1, :), Gyroscope(i, :), Accelerometer(i, :), T, 1, 0.3);
    %q1(i,:) = quatint.m(q1(i-1,:), Gyroscope(i, :), T);
	%q2(i,:) = quatint.q(q2(i-1,:), Gyroscope(i, :), T);
    eul(:,i) = ch_q2eul(q1(:,i));
end


figure('Name', 'Euler Angles');
hold on;
plot(imu.time, eul(1,:), 'r');
plot(imu.time, eul(2,:), 'g');
plot(imu.time, eul(3,:), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

rad2deg( eul(:,end))

