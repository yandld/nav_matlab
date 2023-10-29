%% 新版本 CSV转换工具，适用于 CHCenter > 1.4.1

close all;
clear;
clc;

R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;     % 重力加速度

fullfilename  = "230925165244.csv";

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

[pathstr, file_name, ext] = fileparts(fullfilename);

fprintf("正在读取%s\r\n", fullfilename);
T = readtable(fullfilename); % 读取 CSV 文件
fprintf("读取完成,开始处理帧数据\n");
frame_name = table2cell(T(:,1));

RAWIMUXB = T(strcmp(frame_name, 'RAWIMUX'), 4:10);
RAWIMUXB = cellfun(@str2double, table2cell(RAWIMUXB));

GNSSRCV  = T(strcmp(frame_name, 'GNSSRCV'), 4:34);
GNSSRCV = cellfun(@str2double, table2cell(GNSSRCV));

%清除帧中重复数据，上位机采集失误?
[~, idx] = unique(GNSSRCV(:,1), 'first');  % 获取矩阵A的唯一元素及其索引
GNSSRCV = GNSSRCV(sort(idx), :);

INSPVAXB = T(strcmp(frame_name, 'INSPVAX'), 4:25);
INSPVAXB = cellfun(@str2double, table2cell(INSPVAXB));

fprintf("读取完成\r\n");

fprintf("%-20s: %d帧\n", "INSPVAXB", length(INSPVAXB));
fprintf("%-20s: %d帧\n", "RAWIMUXB",length(RAWIMUXB));
fprintf("%-20s: %d帧\n", "GNSSRCV",length(GNSSRCV));


% IMU数据
data.imu.tow = RAWIMUXB(:, 1);
data.imu.tow = data.imu.tow - data.imu.tow(1);
data.imu.acc = RAWIMUXB(:, 2:4);
data.imu.gyr = RAWIMUXB(:, 5:7);

% 设备上的组合导航数据
data.ins_dev.tow = INSPVAXB(:, 1);
data.ins_dev.tow = data.ins_dev.tow - data.ins_dev.tow(1);
data.ins_dev.ins_sol_stat =INSPVAXB(:, 2);
data.ins_dev.solq_pos =INSPVAXB(:, 3);
data.ins_dev.lat =INSPVAXB(:, 4);
data.ins_dev.lon =INSPVAXB(:, 5);
data.ins_dev.msl =INSPVAXB(:, 6);
data.ins_dev.undulation =INSPVAXB(:, 7);
data.ins_dev.vel_enu =INSPVAXB(:, 8:10);
data.ins_dev.roll =INSPVAXB(:, 11);
data.ins_dev.pitch =INSPVAXB(:, 12);
data.ins_dev.yaw =INSPVAXB(:, 13);
data.ins_dev.pos_enu_std =INSPVAXB(:, 14:16);
data.ins_dev.vel_enu_std =INSPVAXB(:, 17:19);
data.ins_dev.roll_std =INSPVAXB(:, 20);
data.ins_dev.pitch_std =INSPVAXB(:, 21);
data.ins_dev.yaw_std =INSPVAXB(:, 22);

% 超核自定义帧信息，主要传输GNSS数据和一些其他的设备组合导航信息，如零偏等
data.gnss.tow = GNSSRCV(:, 1);
data.gnss.tow = data.gnss.tow - data.gnss.tow(1);
data.gnss.nv = GNSSRCV(:, 2);
data.gnss.nv_heading = GNSSRCV(:, 3);
data.gnss.solq_pos = GNSSRCV(:, 4);
data.gnss.solq_heading = GNSSRCV(:, 5);
data.gnss.diff_age = GNSSRCV(:, 6);
data.gnss.lat = GNSSRCV(:, 7);
data.gnss.lon = GNSSRCV(:, 8);
data.gnss.msl = GNSSRCV(:, 9);
data.gnss.undulation = GNSSRCV(:, 10);
data.gnss.vel_enu = GNSSRCV(:, 11:13);
data.gnss.bl_pitch = GNSSRCV(:, 14);
data.gnss.bl_yaw = GNSSRCV(:, 15);
data.gnss.bl_len = GNSSRCV(:, 16);
data.gnss.bl_pitch_std = GNSSRCV(:, 17);
data.gnss.bl_yaw_std = GNSSRCV(:, 18);
data.gnss.pos_enu_std = GNSSRCV(:, 19:21);
data.gnss.vel_enu_std = GNSSRCV(:, 22:24);

% 里程计数据
data.od.tow = data.gnss.tow;
data.od.speed = GNSSRCV(:, 31);

%% 外插
% GNSS数据对齐到 IMU 时间戳，并移除nan
data.post.interp_gnss_to_imu = interp1(data.gnss.tow, GNSSRCV, data.ins_dev.tow, 'previous', 'extrap');
nan_rows = any(isnan(data.post.interp_gnss_to_imu), 2);
data.post.interp_gnss_to_imu(nan_rows, :) = 0;

% 使用外插据将 调试信息对齐到对应时间戳中
data.ins_dev.ins_wb = data.post.interp_gnss_to_imu(:, 25:27);
data.ins_dev.ins_gb = data.post.interp_gnss_to_imu(:, 28:30);

%  load data.mat


%% 将经纬度转换为ENU, 并外插到与INS_DEV时间戳对齐
fprintf("LLA转换为ENU\n");
lat0 = data.gnss.lat(1);
lon0 = data.gnss.lon(1);
msl0 = data.gnss.msl(1);

%求解ENU 位置
data.gnss.pos_enu = zeros(length(data.gnss.tow), 3);

for i=1:length(data.gnss.tow)
    [data.gnss.pos_enu(i,1), data.gnss.pos_enu(i,2), data.gnss.pos_enu(i,3)] =  ch_LLA2ENU(data.gnss.lat(i)*D2R, data.gnss.lon(i)*D2R, data.gnss.msl(i), lat0*D2R, lon0*D2R, msl0);
end

%求解ENU 位置
data.ins_dev.pos_enu = zeros(length(data.ins_dev.tow), 3);

for i=1:length(data.ins_dev.tow)
[data.ins_dev.pos_enu(i,1), data.ins_dev.pos_enu(i,2), data.ins_dev.pos_enu(i,3)] =  ch_LLA2ENU(data.ins_dev.lat(i)*D2R, data.ins_dev.lon(i)*D2R, data.ins_dev.msl(i), lat0*D2R, lon0*D2R, msl0);
end

%% 外插处理 
fprintf("外插处理\n");
data.post.gnss_pos_enu = interp1(data.gnss.tow, data.gnss.pos_enu, data.imu.tow, 'previous', 'extrap');
data.post.gnss_pos_enu_std = interp1(data.gnss.tow, data.gnss.pos_enu_std, data.imu.tow, 'previous', 'extrap');
data.post.gnss_vel_enu = interp1(data.gnss.tow, data.gnss.vel_enu, data.imu.tow, 'previous', 'extrap');
data.post.gnss_vel_enu_std = interp1(data.gnss.tow, data.gnss.vel_enu_std, data.imu.tow, 'previous', 'extrap');
data.post.ins_dev_pos_enu = interp1(data.ins_dev.tow, data.ins_dev.pos_enu, data.imu.tow, 'previous', 'extrap');
data.post.ins_dev_vel_enu = interp1(data.ins_dev.tow, data.ins_dev.vel_enu, data.imu.tow, 'previous', 'extrap');

%% 打印关键信息
fprintf("%-20s: LLA: %.7f, %.7f, %.1f\n", "起点", lat0, lon0, msl0);
fprintf("%-20s: %.3f s\n", "总计运行时间", max(data.imu.tow) - min(data.imu.tow));
fprintf("%-20s: %.3f s\n", "INS帧平均发送周期", mean(diff(data.ins_dev.tow)));
fprintf("%-20s: %.3f s\n", "IMU帧平均发送周期", mean(diff(data.imu.tow)));
fprintf("%-20s: %.3f s\n", "GNSS帧平均发送周期", mean(diff(data.gnss.tow)));

%% 绘图
set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
set(groot, 'defaultAxesZGrid', 'on');


figure('name', '传感器原始数据');  grid on;
subplot(3,3,1); plot(diff(data.imu.tow), '.-'); title("RAWIMUX 帧时间差分"); xlabel("帧数"); ylabel("s");
subplot(3,3,4); plot(diff(data.ins_dev.tow), '.-'); title("INSPVAXB 帧时间差分"); xlabel("帧数"); ylabel("s");
subplot(3,3,7); plot(diff(data.gnss.tow), '.-'); title("GNSSRCV 帧时间间隔"); xlabel("帧数"); ylabel("s");
subplot(3,3,2); plot(data.imu.tow, data.imu.acc, '.-');title("加速度计"); xlabel("tow"); ylabel("G"); legend("X", "Y", "Z");
subplot(3,3,5); plot(data.imu.tow, data.imu.gyr, '.-'); title("陀螺仪"); xlabel("tow"); ylabel("dps"); legend("X", "Y", "Z");
subplot(3,3,8); plot(data.od.tow, data.od.speed, '.-');  title("里程计"); xlabel("tow"); ylabel("km/h");


figure('name', '位置信息');
subplot(3,2,1); plot(data.imu.tow, data.post.gnss_pos_enu(:,1), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_pos_enu(:,1), '.-'); hold on; title("POS EAST");
subplot(3,2,3); plot(data.imu.tow, data.post.gnss_pos_enu(:,2), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_pos_enu(:,2), '.-'); hold on; title("POS NORTH");
subplot(3,2,5); plot(data.imu.tow, data.post.gnss_pos_enu(:,3), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_pos_enu(:,3), '.-'); hold on; title("POS UP");
subplot(3,2,2); plot(data.imu.tow, data.post.gnss_pos_enu_std(:,1) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.pos_enu_std(:,1), '.-'); hold on; title("POS EAST STD");
subplot(3,2,4); plot(data.imu.tow, data.post.gnss_pos_enu_std(:,2) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.pos_enu_std(:,2), '.-'); hold on; title("POS NORTH STD");
subplot(3,2,6); plot(data.imu.tow, data.post.gnss_pos_enu_std(:,3) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.pos_enu_std(:,3), '.-'); hold on; title("POS UP STD");
axes = findall(gcf, 'Type', 'axes');
for i = 1:length(axes)
    legend(axes(i), "GNSS", "组合导航"); xlabel(axes(i), "tow"); ylabel(axes(i), "m");
end

figure('name', '速度信息');
subplot(3,2,1); plot(data.imu.tow, data.post.gnss_vel_enu(:,1), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_vel_enu(:,1), '.-'); hold on; title("VEL EAST");
subplot(3,2,3); plot(data.imu.tow, data.post.gnss_vel_enu(:,2), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_vel_enu(:,2), '.-'); hold on; title("VEL NORTH");
subplot(3,2,5); plot(data.imu.tow, data.post.gnss_vel_enu(:,3), '.-'); hold on; plot(data.imu.tow, data.post.ins_dev_vel_enu(:,3), '.-'); hold on; title("VEL UP");
subplot(3,2,2); plot(data.imu.tow, data.post.gnss_vel_enu_std(:,1) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.vel_enu_std(:,1), '.-'); hold on; title("VEL EAST STD");
subplot(3,2,4); plot(data.imu.tow, data.post.gnss_vel_enu_std(:,2) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.vel_enu_std(:,2), '.-'); hold on; title("VEL NORTH STD");
subplot(3,2,6); plot(data.imu.tow, data.post.gnss_vel_enu_std(:,3) , '.-'); hold on; plot(data.imu.tow, data.ins_dev.vel_enu_std(:,3), '.-'); hold on; title("VEL UP STD");
axes = findall(gcf, 'Type', 'axes');
for i = 1:length(axes)
    legend(axes(i), "GNSS", "组合导航"); xlabel(axes(i), "tow"); ylabel(axes(i), "m/s");
end


figure('name', '姿态信息');
subplot(3,2,1); plot(data.ins_dev.tow, data.ins_dev.roll, '.-'); hold on; title("Roll");
subplot(3,2,3); plot(data.ins_dev.tow, data.ins_dev.pitch, '.-'); hold on; title("Pitch");
subplot(3,2,5); plot(data.ins_dev.tow, data.ins_dev.yaw, '.-'); hold on; title("Yaw");
subplot(3,2,2); plot(data.ins_dev.tow, data.ins_dev.roll_std, '.-'); hold on; title("Roll STD");
subplot(3,2,4); plot(data.ins_dev.tow, data.ins_dev.pitch_std, '.-'); hold on; title("Pitch STD)");
subplot(3,2,6); plot(data.ins_dev.tow, data.ins_dev.yaw_std, '.-'); hold on; title("Yaw STD");

axes = findall(gcf, 'Type', 'axes');
for i = 1:length(axes)
    legend(axes(i), "组合导航");
    xlabel(axes(i), "tow"); 
    ylabel(axes(i), "deg");
end

figure('name', '零偏信息');
subplot(2,1,1); plot(data.ins_dev.tow, data.ins_dev.ins_wb*R2D, '.-');  title("GYR BIAS"); xlabel("tow"); ylabel("deg"); legend("X", "Y", "Z");
subplot(2,1,2); plot(data.ins_dev.tow, data.ins_dev.ins_gb*1000/GRAVITY, '.-');  title("ACC BIAS"); xlabel("tow"); ylabel("mg"); legend("X", "Y", "Z");

figure('name', '2D轨迹');
subplot(1,1,1);
plot(data.gnss.lon, data.gnss.lat, '.-');
hold on;
plot(data.ins_dev.lon, data.ins_dev.lat, '.-');

title("GNSS位置");
legend("纯GNSS", "设备组合导航");
xlabel("lon(deg)"); ylabel("lat(deg)");

%% 保存数据
fprintf("保存数据...\r\n");
fprintf("保存位置%s/%s\r\n", scriptFolder, fullfile(file_name + ".mat"));
save(fullfile(file_name + ".mat"), 'data');
fprintf("保存完成\r\n");




