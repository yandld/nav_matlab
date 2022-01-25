close all;
clear;
clc;

% [lat, lon, alt, vel, att, pos_std, vel_std, att_std, ins_status, pos_type] = inspvaxa2pvax('./0115/BY.txt');
[lat, lon, alt, vel, att, pos_std, vel_std, att_std, ins_status, pos_type] = inspvaxa2pvax('./0122/BY.txt');
[neu] = pos2xyz(lat, lon, alt);

%%
figure('name', '二维轨迹图');
plot(neu(:, 1), neu(:, 2), 'r.', 'LineWidth', 3); grid on; axis equal;
xlabel('东向距离(m)');
ylabel('北向距离(m)');
title('二维轨迹图');

%%
figure('name', '高度与位置方差曲线');
subplot(2,1,1);
plot(neu(:, 3), 'LineWidth', 1.5); grid on;
xlim([0 length(ins_status)]);
xlabel('时间(s)');
ylabel('高度(m)');
title('高度曲线');
subplot(2,1,2);
plot(pos_std(:, 1), 'LineWidth', 1.5); hold on; grid on;
plot(pos_std(:, 2), 'LineWidth', 1.5);
plot(pos_std(:, 3), 'LineWidth', 1.5);
xlim([0 length(ins_status)]);
xlabel('时间(s)');
ylabel('位置方差(m)');
legend('东向', '北向', '天向', 'Orientation', 'horizontal');
title('位置方差曲线');

%%
figure('name', '速度曲线');
subplot(2,1,1);
plot(vel(:, 1), 'LineWidth', 1.5); hold on; grid on;
plot(vel(:, 2), 'LineWidth', 1.5);
plot(vel(:, 3), 'LineWidth', 1.5);
xlim([0 length(ins_status)]);
ylabel('速度(m/s)');
title('速度曲线');
legend('东向', '北向', '天向', 'Orientation', 'horizontal');
subplot(2,1,2);
plot(vel_std(:, 1), 'LineWidth', 1.5); hold on; grid on;
plot(vel_std(:, 2), 'LineWidth', 1.5);
plot(vel_std(:, 3), 'LineWidth', 1.5);
xlim([0 length(ins_status)]);
xlabel('时间(s)');
ylabel('速度方差(m/s)');
legend('东向', '北向', '天向', 'Orientation', 'horizontal');
title('速度方差曲线');

%%
figure('name', '姿态曲线');
subplot(3,1,1);
plot(att(:, 1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(att(:, 2), 'b', 'LineWidth', 1.5);
xlim([0 length(ins_status)]);
ylabel('姿态(°)');
legend('俯仰', '横滚', 'Orientation', 'horizontal');
subplot(3,1,2);
plot(att(:, 3), 'm', 'LineWidth', 1.5); grid on;
xlim([0 length(ins_status)]);
ylabel('航向(°)');
legend('航向', 'Orientation', 'horizontal');
subplot(3,1,3);
plot(att_std(:, 1), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(att_std(:, 2), 'b', 'LineWidth', 1.5);
plot(att_std(:, 3), 'm', 'LineWidth', 1.5);
xlim([0 length(ins_status)]);
xlabel('时间(s)');
ylabel('方差(°)');
legend('俯仰', '横滚', '航向', 'Orientation', 'horizontal');
suptitle('姿态曲线');

%%
wm = webmap('World Imagery');
wmline(lat, lon, 'Color', 'blue', 'Width', 3, 'OverlayName', 'GNSS');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'
