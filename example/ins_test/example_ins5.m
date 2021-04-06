clear;
clc;
close all;
format long

load filter.mat
%% 读取数据
data = csvread("UranusData.csv", 1, 1);
acc = data(:,2:4);
gyr = data(:,5:7);
mag = data(:,8:10);

% acc = data(:,8:10);
% gyr = data(:,11:13);

q= [1 0 0 0]';
% q   = data(1,4:7)';

Fs = 100; 
dt = 1 / Fs;
N = length(acc);

fprintf("共%d数据，用时:%.3fs\n", N, N/Fs);

gyr = deg2rad(gyr);
acc = acc*9.8;

% 惯导解算, 初始化
p = zeros(3, 1);
v = zeros(3, 1);

for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, -9.8]');
 
    linAcc(i,:) = ch_qmulv(q, acc(i,:));
    linAcc(i,3) = linAcc(i,3) -9.8;
    R(:,:,i) = ch_q2m(q);
end

% 速度
linVel = zeros(size(linAcc));
for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * dt;
end

order = 1;
filtCutOff = 0.1;
%[b, a] = butter(order, (2*filtCutOff)/(1/dt), 'high');
%linVelHP = filtfilt(b, a, linVel);
linVelHP = filter(Hd, linVel);

% 位置
linPos = zeros(size(linVelHP));
for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * dt;
end

%[b, a] = butter(order, (2*filtCutOff)/(1/dt), 'high');
%linPosHP = filtfilt(b, a, linPos);
linPosHP = filter(Hd, linPos);


%% Plot
figure('NumberTitle', 'off', 'Name', '速度');
subplot(2,1,1);
plot(linVel);
title('速度');
legend('X', 'Y', 'Z');
subplot(2,1,2);
plot(linVelHP);
title('HP后速度');
legend('X', 'Y', 'Z');



figure('NumberTitle', 'off', 'Name', '位置');
subplot(2,1,1);
plot(linPos);
title('位置');
legend('X', 'Y', 'Z');
subplot(2,1,2);
plot(linPosHP);
title('HP后位置');
legend('X', 'Y', 'Z');


figure('NumberTitle', 'off', 'Name', '原始数据');
subplot(2,1,1);
plot(acc);
legend("X", "Y", "Z");
title("加速度");
subplot(2,1,2);
plot(gyr);
title("陀螺");
legend("X", "Y", "Z");

% 3D位置
%linPosHP = mag;

figure('NumberTitle', 'off', 'Name', '3D位置');
plot3(linPosHP(1,1), linPosHP(1,2), linPosHP(1,3), '-ks');
hold on;
plot3(linPosHP(:,1), linPosHP(:,2), linPosHP(:,3), '.b');
axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
title('3D位置');
legend('起始', '3D');





% SamplePlotFreq = 4;
% 
% SixDOFanimation(linPosHP, R, ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
%                 'Position', [9 39 400 400], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
%                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/dt) / SamplePlotFreq));            
