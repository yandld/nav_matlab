% 创建测试信号和滤波器
Fs = 1000;           % 采样频率 1kHz
t = 0:1/Fs:1-1/Fs;  % 1秒的时间向量
f1 = 5;             % 低频分量 5Hz
f2 = 50;            % 高频分量 50Hz

% 生成测试信号 = 低频分量 + 高频分量
x = sin(2*pi*f1*t) + 0.5*sin(2*pi*f2*t);

% 设计滤波器
order = 2;          % 二阶滤波器
fc = 20;            % 截止频率 20Hz
Wn = fc/(Fs/2);     % 归一化截止频率

% 创建巴特沃斯高通和低通滤波器
[b_hp,a_hp] = butter(order, Wn, 'high');
[b_lp,a_lp] = butter(order, Wn, 'low');

% 应用滤波器
y_hp = filtfilt(b_hp, a_hp, x);  % 高通滤波
y_lp = filtfilt(b_lp, a_lp, x);  % 低通滤波

% 计算频率响应
[h_hp,w] = freqz(b_hp,a_hp,1024);
[h_lp,~] = freqz(b_lp,a_lp,1024);
freq = w*Fs/(2*pi);

% 绘图
figure('Position', [100 100 1200 800]);

% 时域信号对比
subplot(3,1,1);
plot(t(1:500), x(1:500), 'k', 'LineWidth', 1.5);
hold on;
plot(t(1:500), y_hp(1:500), 'r', 'LineWidth', 1.5);
plot(t(1:500), y_lp(1:500), 'b', 'LineWidth', 1.5);
grid on;
legend('原始信号', '高通滤波', '低通滤波');
title('时域信号对比 (前500个采样点)');
xlabel('时间 (s)');
ylabel('幅度');

% 幅频响应
subplot(3,1,2);
semilogx(freq, 20*log10(abs(h_hp)), 'r', 'LineWidth', 1.5);
hold on;
semilogx(freq, 20*log10(abs(h_lp)), 'b', 'LineWidth', 1.5);
grid on;
legend('高通滤波器', '低通滤波器');
title('幅频响应');
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');

% 相频响应
subplot(3,1,3);
semilogx(freq, unwrap(angle(h_hp))*180/pi, 'r', 'LineWidth', 1.5);
hold on;
semilogx(freq, unwrap(angle(h_lp))*180/pi, 'b', 'LineWidth', 1.5);
grid on;
legend('高通滤波器', '低通滤波器');
title('相频响应');
xlabel('频率 (Hz)');
ylabel('相位 (度)');

% 添加截止频率垂直线
for i = 2:3
    subplot(3,i);
    xline(fc, '--k', sprintf('fc = %d Hz', fc));
end
