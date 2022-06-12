clear;
clc;
close all;






% load('ch100.mat');
% gyroReading = imu.gyr * 3600;
% accelReading = imu.acc *  1000;
% Fs = 400;

load('ch104.mat');
gyroReading = gyroReading * 3600;
accelReading = accReading *  1000;
Fs = 100;


(length(gyroReading) / Fs) / 3600

% load('../stim300.mat');
% gyroReading = imu.gyr;
% accelReading = imu.acc;
% 
% %转换为 deg/h,  m/s^(2)
% gyroReading = gyroReading*3600;
% accelReading = accelReading *1000;
% Fs = 100;

figure

[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,1) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,2) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,3) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
title('Gyroscope -- Allan variance');
legend('X','Y','Z');
xlabel('\tau(s)');
ylabel('Allan standard deviation [deg/h]');
grid on



figure
[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,1) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,2) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,3) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
legend('X','Y','Z');
title('Accelerometer -- Allan variance');
xlabel('\tau(s)');
ylabel('Allan standard deviation [mg]');
grid on

