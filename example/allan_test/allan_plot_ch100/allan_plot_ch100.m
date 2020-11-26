clear;
clc;
close all;

%%  ch00   deg/s    m/s^(2)
load('ch100.mat');
gyroReading = (gyroReading)*3600;
accelReading = accelReading *  1000;
Fs = 400;

figure

[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,1) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,2) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau , ~, ~, ~] = ch_allan(gyroReading(:,3) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);
title('Gyroscope -- Allan variance');
legend('X','Y','Z');
xlabel('\tau(s)');
ylabel('Allan standard deviation [deg/h]');
grid on
axis equal
    
figure
[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,1) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,2) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau , ~, ~, ~] = ch_allan(accelReading(:,3) , Fs, false);
adev  = sqrt(avar);
loglog(tau, adev);
legend('X','Y','Z');
title('Accelerometer -- Allan variance');
xlabel('\tau(s)');
ylabel('Allan standard deviation [mg]');
grid on
axis equal

