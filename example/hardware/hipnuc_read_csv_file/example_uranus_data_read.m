clear;
clc;
close all;

%% 读取数据
data = csvread("UranusData.csv", 1, 1);
gyroReading = data(:,5:7);
accReading = data(:,2:4);

figure;
plot(accReading);
legend("X", "Y", "Z"); title("ACC");

figure;
plot(gyroReading);
legend("X", "Y", "Z"); title("GYR");


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
