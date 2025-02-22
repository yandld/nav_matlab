clear;
close all;
clc;

%%加载升沉数据
data = readtable("100s_110mm_0.2Hz_sin5.csv");

%%加载数据
GRAVITY = 9.81;
n = height(data);
acc_n = zeros(n,3);
Fs = 100;   
dt = 1/Fs;  
t = (0:n-1) * dt;

% 计算导航系加速度
Qb2n = [data.quat_w, data.quat_x,data.quat_y,data.quat_z];
acc_b = [data.acc_x, data.acc_y, data.acc_z] * GRAVITY;

for i = 1:n
    acc_n(i,:) = qmulv(Qb2n(i,:) , acc_b(i,:));
    acc_n(i,3) = acc_n(i,3) - GRAVITY;
end

%% 带通滤波器方案 - 更温和的参数
% 1. 降低阶数
% 2. 扩大带宽
% 3. 将中心频率稍微偏离0.2Hz以补偿相位
order = 2;  % 使用2阶滤波器
Fc_center = 0.1;
bandwidth = 1;  % 显著增加带宽，减少滤波强度
Fc_band = [max(0.05, Fc_center-bandwidth/2), Fc_center+bandwidth/2];
[b_band, a_band] = butter(order, Fc_band/(Fs/2), 'bandpass');
y_bandpass = filter(b_band, a_band, acc_n);

%% 绘图
figure('Name', 'Filter Comparison');
subplot(2,1,1);
plot(t, acc_n(:,3), 'b.', 'MarkerSize', 2, 'DisplayName', 'Raw');
hold on;
plot(t, y_bandpass(:,3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Bandpass');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Full Signal Comparison');
legend('show');
grid on;
xlim([0 50]);

% 放大显示2-4秒的细节
subplot(2,1,2);
plot(t, acc_n(:,3), 'b.', 'MarkerSize', 2, 'DisplayName', 'Raw');
hold on;
plot(t, y_bandpass(:,3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Bandpass');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Detailed View (2-4 seconds)');
legend('show');
grid on;
xlim([2 4]);

% 显示滤波器的频率响应
figure('Name', 'Filter Response');
[h,w] = freqz(b_band,a_band,1024);
f = w*Fs/(2*pi);
subplot(2,1,1);
plot(f,20*log10(abs(h)));
grid on;
title('Magnitude Response');
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
xlim([0 2]);

subplot(2,1,2);
plot(f,unwrap(angle(h))*180/pi);
grid on;
title('Phase Response');
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
xlim([0 2]);
