clear;
clc;
close all;


%%
file = 'bmi088x4.mat';

load(file);
gyroReading = gyroReading * 3600;
accelReading = accReading *  1000;
Fs = 100;
N = length(gyroReading);

fprintf("数据共:%d，采样率:%dHz，时间:%d hr\r\n", N, Fs, N / Fs/3600);

%%
half = zeros(ceil(N/2), 3);

for i=1:N/2
    half(i,:) = gyroReading(2*i-1,:) + gyroReading(2*i,:)*1;
end
Fs = 50;

%%
figure
[avar, tau] = ch_allan(gyroReading(:,1) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau] = ch_allan(gyroReading(:,2) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau] = ch_allan(gyroReading(:,3) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
title('Gyroscope -- Allan variance');
legend('X','Y','Z');
xlabel('\tau(s)');
ylabel('Allan standard deviation [deg/h]');
grid on



figure
[avar, tau] = ch_allan(accelReading(:,1) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
hold on;
[avar, tau] = ch_allan(accelReading(:,2) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);

[avar, tau] = ch_allan(accelReading(:,3) , Fs);
adev  = sqrt(avar);
loglog(tau, adev);
legend('X','Y','Z');
title('Accelerometer -- Allan variance');
xlabel('\tau(s)');
ylabel('Allan standard deviation [mg]');
grid on

