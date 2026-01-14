close all;
clear;
clc;

R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;     % 重力加速度

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

fullfilename  = "HI83-246-024126.csv";  % 修改为你的文件名

% 获取输入文件的路径和文件名
[inputFolder, file_name, ext] = fileparts(fullfilename);
outputFolder = fullfile(scriptFolder, file_name);

fprintf("正在读取%s\r\n", fullfilename);
T = readtable(fullfilename); % 读取 CSV 文件
fprintf("读取完成,开始处理帧数据\n");

% HI83格式已经是纯数据，不需要frame_name过滤

% 剔除所有经纬度不合法的地方
valid_indices = T.ins_lat < 1;
T(valid_indices, :) = [];

% 提取GNSS数据
rows_to_keep = true(height(T), 1);

% 遍历数据，删除相同经纬高的行
i = 1;
while i <= height(T)
    % 获取当前的经纬高值
    current_lat = T.gnss_lat(i);
    current_lon = T.gnss_lon(i);
    current_alt = T.gnss_msl(i);

    % 找到连续相同经纬高的行
    j = i + 1;
    while j <= height(T) && T.gnss_lat(j) == current_lat && T.gnss_lon(j) == current_lon && T.gnss_msl(j) == current_alt
        rows_to_keep(j) = false;
        j = j + 1;
    end

    % 移动到下一组
    i = j;
end

gnss = T(rows_to_keep, :);

valid_indices = (gnss.gnss_lat < 1 | gnss.gnss_lon < 1);
gnss(valid_indices, :) = [];

N = height(T);
fprintf("共: %d帧, %d个经纬度无效数据已经被剔除\n", N, sum(valid_indices));

N = height(gnss);
fprintf("共: %d帧 GNSS 数据\n", N);

%% UTC时间转换为GPS周和周内秒
% UTC格式: YYMMDD-HHMMSS.sss (例如: 260113-012046.539)
fprintf("转换UTC时间为GPS周和周内秒...\n");

% 解析UTC时间字符串
utc_str = string(T.utc);
gps_week = zeros(height(T), 1);
gps_tow = zeros(height(T), 1);

for i = 1:height(T)
    utc = char(utc_str(i));
    
    % 解析日期和时间: YYMMDD-HHMMSS.sss
    year = str2double(['20' utc(1:2)]);  % YY -> YYYY
    month = str2double(utc(3:4));
    day = str2double(utc(5:6));
    hour = str2double(utc(8:9));
    minute = str2double(utc(10:11));
    second = str2double(utc(12:end));
    
    % 转换为GPS周和周内秒
    [gps_week(i), gps_tow(i)] = utc2gps(year, month, day, hour, minute, second);
end

% GNSS数据的GPS周和周内秒
gnss_utc_str = string(gnss.utc);
gnss_gps_week = zeros(height(gnss), 1);
gnss_gps_tow = zeros(height(gnss), 1);

for i = 1:height(gnss)
    utc = char(gnss_utc_str(i));
    
    year = str2double(['20' utc(1:2)]);
    month = str2double(utc(3:4));
    day = str2double(utc(5:6));
    hour = str2double(utc(8:9));
    minute = str2double(utc(10:11));
    second = str2double(utc(12:end));
    
    [gnss_gps_week(i), gnss_gps_tow(i)] = utc2gps(year, month, day, hour, minute, second);
end

%% 构建数据结构

% IMU数据 - 使用GPS周内秒
data.imu.week = gps_week;
data.imu.tow = gps_tow;
data.imu.acc = [T.acc_x, T.acc_y, T.acc_z];
data.imu.gyr = [T.gyr_x, T.gyr_y, T.gyr_z];

% 设备上的组合导航数据
data.dev.week = gps_week;
data.dev.tow = gps_tow;
data.dev.ins_lat = T.ins_lat;
data.dev.ins_lon = T.ins_lon;
data.dev.ins_msl = T.ins_msl;
data.dev.vel_enu = [T.ins_vel_e, T.ins_vel_n, T.ins_vel_u];
data.dev.kf_wb = [T.kf_wb_x, T.kf_wb_y, T.kf_wb_z];
data.dev.kf_gb = [T.kf_gb_x, T.kf_gb_y, T.kf_gb_z];
data.dev.roll = T.roll;
data.dev.pitch = T.pitch;
% HI83使用imu_yaw作为航向角，需要取反并转换到0-360度
data.dev.yaw = mod(-T.imu_yaw + 360, 360);

% GNSS
data.gnss.week = gnss_gps_week;
data.gnss.tow = gnss_gps_tow;
data.gnss.solq_pos = gnss.solq_pos;
data.gnss.solq_pos2 = T.solq_pos;  % 全部数据的solq_pos
data.gnss.solq_heading = gnss.solq_heading;
data.gnss.solq_heading2 = T.solq_heading;  % 全部数据的solq_heading
data.gnss.lat = gnss.gnss_lat;
data.gnss.lon = gnss.gnss_lon;
data.gnss.msl = gnss.gnss_msl;
data.gnss.vel_enu = [gnss.gnss_vel_e, gnss.gnss_vel_n, gnss.gnss_vel_u];
data.gnss.gnss_pos_std_n = gnss.gnss_pos_std_n;
data.gnss.gnss_vel_std_n = gnss.gnss_vel_std_n;

% 双天线数据 - 使用基线长度和角度
data.gnss.dual_base_len = gnss.gnss_dual_base_len;
data.gnss.dual_pitch_deg = gnss.gnss_dual_pitch_deg;
data.gnss.dual_heading_deg = gnss.gnss_dual_heading_deg;

data.gnss.nv = gnss.nv;
data.gnss.gnss_delay = gnss.undulation;

% 里程计
data.od.tow = T.sys_time / 1000;  % sys_time转换为秒
data.od.speed = T.od_vel;

% 相对时间
time0 = data.imu.tow(1);
data.imu.tow = data.imu.tow - time0;
data.dev.tow = data.dev.tow - time0;
data.gnss.tow = data.gnss.tow - time0;
data.od.tow = data.od.tow - data.od.tow(1);

%% 将经纬度转换为ENU
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
data.dev.pos_enu = zeros(length(data.dev.tow), 3);

for i=1:length(data.dev.tow)
    [data.dev.pos_enu(i,1), data.dev.pos_enu(i,2), data.dev.pos_enu(i,3)] =  ch_LLA2ENU(data.dev.ins_lat(i)*D2R, data.dev.ins_lon(i)*D2R, data.dev.ins_msl(i), lat0*D2R, lon0*D2R, msl0);
end

%% 打印关键信息
fprintf("%-20s: LLA: %.7f, %.7f, %.1f\n", "起点", lat0, lon0, msl0);
fprintf("%-20s: GPS周 %d\n", "GPS周", data.imu.week(1));
fprintf("%-20s: %.3f s\n", "总计运行时间", max(data.imu.tow) - min(data.imu.tow));
fprintf("%-20s: %.3f s\n", "INS帧平均发送周期", mean(diff(data.imu.tow)));
fprintf("%-20s: %.3f s\n", "IMU帧平均发送周期", mean(diff(data.imu.tow)));
fprintf("%-20s: %.3f s\n", "GNSS帧平均发送周期", mean(diff(data.gnss.tow)));

% 修改保存数据的路径
fprintf("保存数据...\r\n");
fprintf("保存位置%s\r\n", fullfile(outputFolder, file_name + ".mat"));
% 检查目录是否存在，如果不存在则创建它
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end
save(fullfile(outputFolder, file_name + ".mat"), 'data');
fprintf("保存完成\r\n");

%% 转换为kml
[filepath, name, ~] = fileparts(fullfilename);

rgbColor = [255, 0, 0];
kmlFileName = fullfile(outputFolder, file_name + "_GNSS.kml");
generateKmlFiles(kmlFileName, data.gnss.lat, data.gnss.lon, 20, rgbColor);

rgbColor = [0, 255, 0];
kmlFileName = fullfile(outputFolder, file_name + "_DEV.kml");
generateKmlFiles(kmlFileName, data.dev.ins_lat, data.dev.ins_lon, 20, rgbColor);

%% 绘图
set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
set(groot, 'defaultAxesZGrid', 'on');

figure('name', '传感器原始数据');  grid on;
subplot(3,3,1); plot(diff(data.imu.tow), '.-'); title("RAWIMUX 帧时间差分"); xlabel("帧数"); ylabel("s"); xlim tight;
subplot(3,3,7); plot(diff(data.gnss.tow), '.-'); title("GNSSRCV 帧时间间隔"); xlabel("帧数"); ylabel("s"); xlim tight;
subplot(3,3,2); plot(data.imu.tow, data.imu.acc, '.-');title("加速度计"); xlabel("tow"); ylabel("G"); legend("X", "Y", "Z"); xlim tight;
subplot(3,3,5); plot(data.imu.tow, data.imu.gyr, '.-'); title("陀螺仪"); xlabel("tow"); ylabel("dps"); legend("X", "Y", "Z"); xlim tight;
subplot(3,3,8); plot(data.od.tow, data.od.speed, '.-');  title("里程计"); xlabel("tow"); ylabel("km/h"); xlim tight;
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

figure('name', '纯GNSS标准差');
subplot(2,2,1); plot(data.gnss.tow, data.gnss.gnss_pos_std_n , '.-');  title("GNSS接收机给出的 pos\_std(ENU模)"); hold on;
subplot(2,2,2);plot(data.gnss.tow, data.gnss.gnss_vel_std_n, '.-'); title("GNSS接收机给出的 vel\_std(ENU模)"); hold on;
subplot(2,2,3);plot(data.gnss.tow, data.gnss.solq_pos, '.-'); title("solq\_pos"); hold on;

%%
figure('name', '位置与速度');
gnss_pos_enu_interp = interp1(data.gnss.tow, data.gnss.pos_enu, data.imu.tow, 'previous', 'extrap');
gnss_vel_enu_interp = interp1(data.gnss.tow, data.gnss.vel_enu, data.imu.tow, 'previous', 'extrap');

subplot(3,2,2); plot(data.imu.tow, gnss_pos_enu_interp(:,1), '.-'); hold on; plot(data.imu.tow, data.dev.pos_enu(:,1), '.-'); hold on; title("Postion East"); xlim tight;
subplot(3,2,4); plot(data.imu.tow, gnss_pos_enu_interp(:,2), '.-'); hold on; plot(data.imu.tow, data.dev.pos_enu(:,2), '.-'); hold on; title("Postion North"); xlim tight;
subplot(3,2,6); plot(data.imu.tow, gnss_pos_enu_interp(:,3), '.-'); hold on; plot(data.imu.tow, data.dev.pos_enu(:,3), '.-'); hold on; title("Postion Up"); xlim tight;
subplot(3,2,1); plot(data.imu.tow, gnss_vel_enu_interp(:,1), '.-'); hold on; plot(data.imu.tow, data.dev.vel_enu(:,1), '.-'); hold on; title("Vel East"); xlim tight;
subplot(3,2,3); plot(data.imu.tow, gnss_vel_enu_interp(:,2), '.-'); hold on; plot(data.imu.tow, data.dev.vel_enu(:,2), '.-'); hold on; title("Vel North"); xlim tight;
subplot(3,2,5); plot(data.imu.tow, gnss_vel_enu_interp(:,3), '.-'); hold on; plot(data.imu.tow, data.dev.vel_enu(:,3), '.-'); hold on; title("Vel Up"); xlim tight;
legend("纯GNSS", "设备组合导航");

%%
figure('name', '姿态信息');
subplot(2,2,1); plot(data.dev.tow, data.dev.roll, '.-'); hold on; title("Roll"); xlim tight;
subplot(2,2,2); plot(data.dev.tow, data.dev.pitch, '.-'); hold on; title("Pitch"); xlim tight;
subplot(2,2,3); plot(data.dev.tow, data.dev.yaw, '.-'); hold on; title("Yaw"); xlim tight;

figure('name', '零偏信息');
subplot(2,1,1); plot(data.dev.tow, data.dev.kf_wb*R2D, '.-');  title("GYR BIAS"); xlabel("tow"); ylabel("deg"); legend("X", "Y", "Z"); xlim tight;
subplot(2,1,2); plot(data.dev.tow, data.dev.kf_gb*1000/GRAVITY, '.-');  title("ACC BIAS"); xlabel("tow"); ylabel("mg"); legend("X", "Y", "Z"); xlim tight;

figure('name', '纯GNSS位置');
subplot(3,1,1);
plot(data.gnss.tow, data.gnss.pos_enu(:,1), '.-');
title('Pos East'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB'); xlim tight;
subplot(3,1,2);
plot(data.gnss.tow, data.gnss.pos_enu(:,2), '.-'); hold on;
title('Pos North'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB'); xlim tight;
subplot(3,1,3);
plot(data.gnss.tow, data.gnss.pos_enu(:,3), '.-'); hold on;
title('Pos Up'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB'); xlim tight;

%% 
figure('name', '2D轨迹');
plot(data.gnss.pos_enu(:,1), data.gnss.pos_enu(:,2), '.-');
hold on;
plot(data.dev.pos_enu(:,1), data.dev.pos_enu(:,2), '.-');
title("GNSS平面轨迹");
legend("纯GNSS", "设备组合导航");
xlabel("East"); ylabel("North");
axis equal;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 双天线基线信息
figure('name', '双天线基线信息');
subplot(3,1,1);
plot(data.gnss.tow, data.gnss.dual_base_len, '.-');
title('双天线基线长度'); xlabel('时间(s)'); ylabel('m'); xlim tight;
subplot(3,1,2);
plot(data.gnss.tow, data.gnss.dual_pitch_deg, '.-');
title('双天线俯仰角'); xlabel('时间(s)'); ylabel('deg'); xlim tight;
subplot(3,1,3);
plot(data.gnss.tow, data.gnss.dual_heading_deg, '.-');
title('双天线航向角'); xlabel('时间(s)'); ylabel('deg'); xlim tight;

%% 

% 打印保存位置信息
fprintf('所有文件已保存在: %s\n', outputFolder);
winopen(outputFolder);

%% UTC转GPS周和周内秒函数
function [gps_week, gps_tow] = utc2gps(year, month, day, hour, minute, second)
    % GPS起始时间: 1980年1月6日 00:00:00 UTC
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0);
    
    % 当前UTC时间
    utc_time = datetime(year, month, day, hour, minute, second);
    
    % 计算从GPS起始时间到当前时间的秒数
    % 注意：这里没有考虑闰秒，如果需要精确计算需要加上闰秒修正
    elapsed_seconds = seconds(utc_time - gps_epoch);
    
    % GPS周数 (1周 = 604800秒)
    gps_week = floor(elapsed_seconds / 604800);
    
    % 周内秒
    gps_tow = mod(elapsed_seconds, 604800);
end
