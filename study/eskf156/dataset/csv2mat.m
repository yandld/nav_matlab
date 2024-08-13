close all;
clear;
clc;

R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;     % 重力加速度


fullfilename  = "./240718A2/240718A2.csv";

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

[pathstr, file_name, ext] = fileparts(fullfilename);

fprintf("正在读取%s\r\n", fullfilename);
T = readtable(fullfilename); % 读取 CSV 文件
fprintf("读取完成,开始处理帧数据\n");
frame_name = table2cell(T(:,1));

T = T(strcmp(frame_name, 'HI43'), 2:end);

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


% 事件
% data.evt = frame.evt_bit;

% IMU数据
 data.imu.tow = T.gps_tow;
 data.imu.tow = data.imu.tow;
 data.imu.acc = [T.acc_x, T.acc_y, T.acc_z];
 data.imu.gyr = [T.gyr_x, T.gyr_y, T.gyr_z];

% 设备上的组合导航数据
data.dev.tow = data.imu.tow;
data.dev.ins_lat = T.ins_lat;
data.dev.ins_lon = T.ins_lon;
data.dev.ins_msl = T.ins_msl;
data.dev.vel_enu = [T.ins_vel_e, T.ins_vel_n, T.ins_vel_u];
data.dev.kf_wb = [T.kf_wb_x, T.kf_wb_y, T.kf_wb_z];
data.dev.kf_gb = [T.kf_gb_x, T.kf_gb_y, T.kf_gb_z];
data.dev.kf_p_att = [T.kf_p_att_x, T.kf_p_att_y, T.kf_p_att_z];
data.dev.kf_p_pos = [T.kf_p_pos_x, T.kf_p_pos_y, T.kf_p_pos_z];
data.dev.kf_p_vel = [T.kf_p_vel_x, T.kf_p_vel_y, T.kf_p_vel_z];
data.dev.kf_p_wb = [T.kf_p_wb_x, T.kf_p_wb_y, T.kf_p_wb_z];
data.dev.kf_p_gb = [T.kf_p_gb_x, T.kf_p_gb_y, T.kf_p_gb_z];
data.dev.roll = T.roll;
data.dev.pitch = T.pitch;
data.dev.yaw = T.yaw;

% GNSS
data.gnss.tow = gnss.gps_tow;
data.gnss.solq_pos = gnss.solq_pos;
data.gnss.solq_heading = gnss.solq_heading;
data.gnss.lat = gnss.gnss_lat;
data.gnss.lon = gnss.gnss_lon;
data.gnss.msl = gnss.gnss_msl;
data.gnss.vel_enu = [gnss.gnss_vel_e, gnss.gnss_vel_n, gnss.gnss_vel_u];
data.gnss.gnss_pos_std_n = gnss.gnss_pos_std_n;
data.gnss.gnss_vel_std_n = gnss.gnss_vel_std_n;
data.gnss.nv = gnss.nv;
data.gnss.hdop = gnss.hdop;

% 里程计
data.od.tow = data.imu.tow;
data.od.speed = T.pc_counter;

% 相对时间
time0 = data.imu.tow(1);
data.imu.tow = data.imu.tow - time0;
data.dev.tow = data.dev.tow - time0;
data.gnss.tow = data.gnss.tow - time0;
data.od.tow = data.od.tow - time0;

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
data.dev.pos_enu = zeros(length(data.dev.tow), 3);

for i=1:length(data.dev.tow)
[data.dev.pos_enu(i,1), data.dev.pos_enu(i,2), data.dev.pos_enu(i,3)] =  ch_LLA2ENU(data.dev.ins_lat(i)*D2R, data.dev.ins_lon(i)*D2R, data.dev.ins_msl(i), lat0*D2R, lon0*D2R, msl0);
end

%% 打印关键信息
fprintf("%-20s: LLA: %.7f, %.7f, %.1f\n", "起点", lat0, lon0, msl0);
fprintf("%-20s: %.3f s\n", "总计运行时间", max(data.imu.tow) - min(data.imu.tow));
fprintf("%-20s: %.3f s\n", "INS帧平均发送周期", mean(diff(data.imu.tow)));
fprintf("%-20s: %.3f s\n", "IMU帧平均发送周期", mean(diff(data.imu.tow)));
fprintf("%-20s: %.3f s\n", "GNSS帧平均发送周期", mean(diff(data.gnss.tow)));

%% 保存数据
fprintf("保存数据...\r\n");
fprintf("保存位置%s/\r\n", fullfile(pathstr, file_name + ".mat"));
save(fullfile(pathstr, file_name + ".mat"), 'data');
fprintf("保存完成\r\n");



%% 转换为kml
[filepath, name, ~] = fileparts(fullfilename);

rgbColor = [255, 0, 0];
kmlFileName = fullfile(filepath, name + "_GNSS.kml");
generateKmlFiles(kmlFileName, data.gnss.lat, data.gnss.lon, 20, rgbColor);

rgbColor = [0, 255, 0];
kmlFileName = fullfile(filepath, name + "_DEV.kml");
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
subplot(2,2,1); plot(data.gnss.tow, data.gnss.gnss_pos_std_n , '.-');  title("GNSS接收机给出的 pos_std(ENU模)"); hold on; 
subplot(2,2,2);plot(data.gnss.tow, data.gnss.gnss_vel_std_n, '.-'); title("GNSS接收机给出的 vel_std(ENU模)"); hold on;
subplot(2,2,3);plot(data.gnss.tow, data.gnss.solq_pos, '.-'); title("solq_pos"); hold on;


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


figure('name', '设备KF收敛状态');
subplot(2,3,1); plot(data.imu.tow, data.dev.kf_p_att, '.-'); title("KF Att");
subplot(2,3,2); plot(data.imu.tow, data.dev.kf_p_pos, '.-'); title("KF Postion");
subplot(2,3,3); plot(data.imu.tow, data.dev.kf_p_vel, '.-'); title("KF Velicty");
subplot(2,3,4); plot(data.imu.tow, data.dev.kf_p_wb*R2D, '.-'); title("KF Wb"); ylabel("deg");
subplot(2,3,5); plot(data.imu.tow, data.dev.kf_p_gb / 9.8 * 1000, '.-'); title("KF Gb"); ylabel("mg");
legend("X", "Y", "Z");

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


figure('name', '2D轨迹');
plot(data.gnss.pos_enu(:,1), data.gnss.pos_enu(:,2), '.-');
hold on;
plot(data.dev.pos_enu(:,1), data.dev.pos_enu(:,2), '.-');
title("GNSS平面轨迹");
legend("纯GNSS", "设备组合导航");
xlabel("East"); ylabel("North");
axis equal;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);


