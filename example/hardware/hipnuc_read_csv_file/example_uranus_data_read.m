clear;
clc;
close all;

%% 读取数据
data = csvread("20240206_IMU2.csv", 1, 1);
gyroReading = data(:,6:8);
accReading = data(:,3:5);

% 假设的采样率
sampleRate = 500;  % 例如，100 Hz

% 确保数据长度是sampleRate的整数倍
len = floor(size(accReading, 1) / sampleRate) * sampleRate;

% 重塑和计算均值
% 对于加速度计数据
accReshaped = reshape(accReading(1:len, :), sampleRate, [], 3);
accMean = squeeze(mean(accReshaped, 1));
accMean(:,3) = accMean(:,3) -1;

% 对于陀螺仪数据
gyroReshaped = reshape(gyroReading(1:len, :), sampleRate, [], 3);
gyroMean = squeeze(mean(gyroReshaped, 1));

% 绘制加速度计数据
figure;
plot(accMean);
legend("X", "Y", "Z");
title("ACC Mean Per Second");

% 绘制陀螺仪数据
figure;
plot(gyroMean);
legend("X", "Y", "Z");
title("GYR Mean Per Second");





% X = accReading(:,3);
% 
% Fs = 400;            % Sampling frequency                    
% T = 1/Fs;             % Sampling period       
% L = length(accReading);             % Length of signal
% t = (0:L-1)*T;        % Time vector
% 
% 
% 
% Y = fft(X);
% P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% f = Fs*(0:(L/2))/L;
% figure;
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f (Hz)')
% ylabel('|P1(f)|')


%plot(gyroReading,'.-');

% 
%  z = gyroReading(:,3);
%  N = length(z);
%  
% sqrt( (dot(z, z) - sum(z)^(2) / N) / (N - 1) )
% 
% gstd =  std(gyroReading)*1;
% 
% fprintf("陀螺std:  X:%.3f, Y:%.3f, Z:%.3f\n", gstd(1), gstd(2), gstd(3) );

%保存为文本文件
% save static_gyr.txt gyroReading -ascii
