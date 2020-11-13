clear;
clc;
close all;
%%


%load('LoggedSingleAxisGyroscope', 'omega', 'Fs')

load('adis16488_gyr.mat');
omega = omega(:,1);
omega = deg2rad(omega);

[avar1, tau1 , N, K, B] = ch_allan(omega, Fs, true);

fprintf('零偏不稳定性                                    %frad/s                    或   %fdeg/h \n', B, rad2deg(B)*3600);
fprintf('角度随机游走(ARW, Noise density)    %f(rad/s)/sqrt(Hz)    或  %f deg/sqrt(h)\n', N, rad2deg(N)*3600^(0.5));
fprintf('角速率游走                                       %f(rad/s)sqrt(Hz)      或  %f deg/h/sqrt(h)\n', K, rad2deg(K) * (3600^(1.5)));



%% 用仿真数据测

% L = 2160000;
% 
% gyro = gyroparams('NoiseDensity', N, 'RandomWalk', K,'BiasInstability', B);
% 
% 
% acc = zeros(L, 3);
% angvel = zeros(L, 3);
% imu = imuSensor('SampleRate', Fs, 'Gyroscope', gyro);
% [~, omega] = imu(acc, angvel);
% omega = omega(:,1);
% 
% 
% [avar2, tau2,  N, K, B] = ch_allan(omega, Fs, true);
% 
% fprintf('零偏不稳定性                                    %frad/s                    或   %fdeg/h \n', B, rad2deg(B)*3600);
% fprintf('角度随机游走(ARW, Noise density)    %f(rad/s)/sqrt(Hz)    或  %f deg/sqrt(h)\n', N, rad2deg(N)*3600^(0.5));
% fprintf('角速率游走                                       %f(rad/s)sqrt(Hz)      或  %f deg/h/sqrt(h)\n', K, rad2deg(K) * (3600^(1.5)));

