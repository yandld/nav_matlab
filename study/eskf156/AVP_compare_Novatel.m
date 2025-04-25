close all;
clear;
filename = 'C:\Users\60265\Desktop\hi32_data\2025-4-15\2025-415.csv';
novatelData = readtable(filename);
filename2 = 'D:\WANGFENG\CHAOHE\CHCenter (2)\log\HI43-4-15-APP245-2-15-07-45.csv';
T = readtable(filename2);
frame_name = table2cell(T(:,1));

CH32Data = T(strcmp(frame_name, 'HI43'), 2:end);
%% 提取NovAtel数据中的时间、位置、速度和姿态信息
novatel_time = novatelData.GPSTime; % GPS秒
novatel_lat = novatelData.Latitude;
novatel_lon = novatelData.Longitude;
novatel_height = novatelData.H_Ell;
novatel_veast = novatelData.VEast;
novatel_vnorth = novatelData.VNorth;
novatel_vup = novatelData.VUp;
novatel_heading = novatelData.Heading;
novatel_pitch = novatelData.Pitch;
novatel_roll = novatelData.Roll;

% 提取CH32数据中的时间、位置、速度和姿态信息
ch32_time = CH32Data.gps_tow - 0.01; % GPS秒
ch32_time = round(ch32_time * 100) /100;
ch32_lat = CH32Data.ins_lat;
ch32_lon = CH32Data.ins_lon;
ch32_height = CH32Data.ins_msl;
ch32_veast = CH32Data.ins_vel_e;
ch32_vnorth = CH32Data.ins_vel_n;
ch32_vup = CH32Data.ins_vel_u;
ch32_roll = CH32Data.roll;
ch32_pitch = CH32Data.pitch;
ch32_yaw = CH32Data.yaw; % CH32中使用yaw表示heading

%% 时间匹配
% 查找最接近的时间点进行数据对比
matched_indices_novatel = zeros(length(ch32_time), 1);
matched_indices_ch32 = zeros(length(ch32_time), 1);
matched_count = 0;
start_point = 18000;%设置开始对比的历元，便于事后处理结果收敛
end_point = length(ch32_time) - 45000;
time_threshold = 0.003; % 时间匹配阈值，单位秒
for i = start_point:end_point
    [min_diff, idx] = min(abs(novatel_time - ch32_time(i)));
    if min_diff <= time_threshold
        matched_count = matched_count + 1;
        matched_indices_novatel(matched_count) = idx;
        matched_indices_ch32(matched_count) = i;
    end
end

%%
matched_indices_novatel = matched_indices_novatel(1:matched_count);
matched_indices_ch32 = matched_indices_ch32(1:matched_count);

fprintf('找到 %d 个匹配的时间点\n', matched_count);

if matched_count == 0
    error('没有找到匹配的时间点，请检查数据时间是否一致');
end

nov_time_matched = novatel_time(matched_indices_novatel);
nov_lat_matched = novatel_lat(matched_indices_novatel);
nov_lon_matched = novatel_lon(matched_indices_novatel);
nov_height_matched = novatel_height(matched_indices_novatel);
nov_veast_matched = novatel_veast(matched_indices_novatel);
nov_vnorth_matched = novatel_vnorth(matched_indices_novatel);
nov_vup_matched = novatel_vup(matched_indices_novatel);
nov_heading_matched = novatel_heading(matched_indices_novatel);
nov_pitch_matched = novatel_pitch(matched_indices_novatel);
nov_roll_matched = novatel_roll(matched_indices_novatel);

ch32_time = ch32_time(matched_indices_ch32);
ch32_lat = ch32_lat(matched_indices_ch32);
ch32_lon = ch32_lon(matched_indices_ch32);
ch32_height = ch32_height(matched_indices_ch32);
ch32_veast = ch32_veast(matched_indices_ch32);
ch32_vnorth = ch32_vnorth(matched_indices_ch32);
ch32_vup = ch32_vup(matched_indices_ch32);
ch32_yaw = ch32_yaw(matched_indices_ch32);
ch32_pitch = ch32_pitch(matched_indices_ch32);
ch32_roll = ch32_roll(matched_indices_ch32);
%%
%% 位置转换（从度到米）
% 使用简单的度到米的转换（在小范围内有效）
% 地球半径（米）
earth_radius = 6378137.0;

% 计算参考点（使用第一个匹配点作为参考）
ref_lat = nov_lat_matched(1);
ref_lon = nov_lon_matched(1);

% 转换纬度差为北向距离（米）
nov_north_m = (nov_lat_matched - ref_lat) * (pi/180) * earth_radius;
ch32_north_m = (ch32_lat - ref_lat) * (pi/180) * earth_radius;

% 转换经度差为东向距离（米）
% 考虑到纬度影响经度的实际距离
cos_lat = cos(ref_lat * pi/180);
nov_east_m = (nov_lon_matched - ref_lon) * (pi/180) * earth_radius * cos_lat;
ch32_east_m = (ch32_lon - ref_lon) * (pi/180) * earth_radius * cos_lat;

% 高度已经是米，不需要转换
nov_up_m = nov_height_matched;
ch32_up_m = ch32_height;

%% 计算位置差异
pos_diff_east = ch32_east_m - nov_east_m;
pos_diff_north = ch32_north_m - nov_north_m;
pos_diff_up = ch32_up_m - nov_up_m;

% 水平位置差异（平面距离）
pos_diff_horizontal = sqrt(pos_diff_east.^2 + pos_diff_north.^2);

%% 计算速度差异
vel_diff_east = ch32_veast - nov_veast_matched;
vel_diff_north = ch32_vnorth - nov_vnorth_matched;
vel_diff_up = ch32_vup - nov_vup_matched;

% 水平速度差异
vel_diff_horizontal = sqrt(vel_diff_east.^2 + vel_diff_north.^2);

%% 计算姿态差异
% 注意：航向角可能需要特殊处理，因为它是循环的（0-360度）
% 计算最小角度差
heading_diff = ch32_yaw - nov_heading_matched;
heading_diff = mod(heading_diff + 180, 360) - 180; % 确保差值在-180到180度之间

pitch_diff = ch32_pitch - nov_pitch_matched;
roll_diff = ch32_roll - nov_roll_matched;

%% 绘图
% 创建时间轴（相对于第一个匹配点）
time_axis = nov_time_matched - nov_time_matched(1);

% 设置图形大小和位置
figure('Position', [50, 50, 1200, 900]);

%% 1. 位置轨迹图（平面图）
subplot(3, 3, 1);
plot(nov_east_m, nov_north_m, 'b-', 'LineWidth', 1.5);
hold on;
plot(ch32_east_m, ch32_north_m, 'r--', 'LineWidth', 1.5);
xlabel('东向位置 (m)');
ylabel('北向位置 (m)');
title('位置轨迹对比');
legend('NovAtel', 'CH32');
grid on;
axis equal;

%% 2. 位置差异随时间变化
subplot(3, 3, 2);
plot(time_axis, pos_diff_east, 'r-', 'LineWidth', 1);
hold on;
plot(time_axis, pos_diff_north, 'g-', 'LineWidth', 1);
plot(time_axis, pos_diff_up, 'b-', 'LineWidth', 1);
plot(time_axis, pos_diff_horizontal, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置差异 (m)');
title('位置差异随时间变化');
legend('东向差异', '北向差异', '高度差异', '水平差异');
grid on;

%% 3. 位置差异直方图
subplot(3, 3, 3);
histogram(pos_diff_horizontal, 50);
xlabel('水平位置差异 (m)');
ylabel('频次');
title('水平位置差异分布');
grid on;

%% 4. 速度对比
subplot(3, 3, 4);
plot(time_axis, nov_veast_matched, 'r-', 'LineWidth', 1);
hold on;
plot(time_axis, ch32_veast, 'r--', 'LineWidth', 1);
plot(time_axis, nov_vnorth_matched, 'g-', 'LineWidth', 1);
plot(time_axis, ch32_vnorth, 'g--', 'LineWidth', 1);
plot(time_axis, nov_vup_matched, 'b-', 'LineWidth', 1);
plot(time_axis, ch32_vup, 'b--', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('速度对比');
legend('NovAtel东向', 'CH32东向', 'NovAtel北向', 'CH32北向', 'NovAtel上向', 'CH32上向');
grid on;

%% 5. 速度差异随时间变化
subplot(3, 3, 5);
plot(time_axis, vel_diff_east, 'r-', 'LineWidth', 1);
hold on;
plot(time_axis, vel_diff_north, 'g-', 'LineWidth', 1);
plot(time_axis, vel_diff_up, 'b-', 'LineWidth', 1);
plot(time_axis, vel_diff_horizontal, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度差异 (m/s)');
title('速度差异随时间变化');
legend('东向差异', '北向差异', '上向差异', '水平差异');
grid on;

%% 6. 速度差异直方图
subplot(3, 3, 6);
histogram(vel_diff_horizontal, 50);
xlabel('水平速度差异 (m/s)');
ylabel('频次');
title('水平速度差异分布');
grid on;

%% 7. 姿态对比
subplot(3, 3, 7);
plot(time_axis, nov_heading_matched, 'r-', 'LineWidth', 1);
hold on;
plot(time_axis, ch32_yaw, 'r--', 'LineWidth', 1);
plot(time_axis, nov_pitch_matched, 'g-', 'LineWidth', 1);
plot(time_axis, ch32_pitch, 'g--', 'LineWidth', 1);
plot(time_axis, nov_roll_matched, 'b-', 'LineWidth', 1);
plot(time_axis, ch32_roll, 'b--', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('姿态角 (deg)');
title('姿态对比');
legend('NovAtel航向', 'CH32航向', 'NovAtel俯仰', 'CH32俯仰', 'NovAtel横滚', 'CH32横滚');
grid on;

%% 8. 姿态差异随时间变化
subplot(3, 3, 8);
plot(time_axis, heading_diff, 'r-', 'LineWidth', 1);
hold on;
plot(time_axis, pitch_diff, 'g-', 'LineWidth', 1);
plot(time_axis, roll_diff, 'b-', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('姿态差异 (deg)');
title('姿态差异随时间变化');
legend('航向差异', '俯仰差异', '横滚差异');
grid on;

%% 9. 姿态差异直方图
subplot(3, 3, 9);
histogram(heading_diff, 50, 'FaceColor', 'r', 'FaceAlpha', 0.3);
hold on;
histogram(pitch_diff, 50, 'FaceColor', 'g', 'FaceAlpha', 0.3);
histogram(roll_diff, 50, 'FaceColor', 'b', 'FaceAlpha', 0.3);
xlabel('姿态差异 (deg)');
ylabel('频次');
title('姿态差异分布');
legend('航向差异', '俯仰差异', '横滚差异');
grid on;

%% 调整整体布局
sgtitle('NovAtel与CH32数据对比分析', 'FontSize', 16);
set(gcf, 'Color', 'w');

% 计算统计信息
pos_rms_h = rms(pos_diff_horizontal);
pos_rms_v = rms(pos_diff_up);
vel_rms_h = rms(vel_diff_horizontal);
vel_rms_v = rms(vel_diff_up);
heading_rms = rms(heading_diff);
pitch_rms = rms(pitch_diff);
roll_rms = rms(roll_diff);

% 打印统计信息
fprintf('统计信息:\n');
fprintf('水平位置RMS: %.3f m\n', pos_rms_h);
fprintf('垂直位置RMS: %.3f m\n', pos_rms_v);
fprintf('水平速度RMS: %.3f m/s\n', vel_rms_h);
fprintf('垂直速度RMS: %.3f m/s\n', vel_rms_v);
fprintf('航向角RMS: %.3f deg\n', heading_rms);
fprintf('俯仰角RMS: %.3f deg\n', pitch_rms);
fprintf('横滚角RMS: %.3f deg\n', roll_rms);

% 保存图像
saveas(gcf, 'novatel_ch32_comparison.png');
