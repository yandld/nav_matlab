close all;
clear;
clc;

format long g;
format compact;

deg = 180/pi;
rad = pi/180;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;

%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)

%% 相关选项及参数设置
opt.alignment_time = 10e2;  % 初始对准时间
opt.sins_enable = false;    % 姿态纯惯性积分
opt.bias_feedback = false;  % IMU零偏反馈
opt.gnss_outage = true;    % 模拟GNSS丢失
opt.gravity_update_enable = true; % 使能重力静止量更新
opt.outage_start = 450;     % 丢失开始时间
opt.outage_stop = 520;      % 丢失结束时间
opt.gnss_intervel = 10;      % GNSS间隔时间，如原始数据为10Hz，那么 gnss_intervel=10 则降频为1Hz
opt.imu_intervel = 1;       % IMU间隔时间，如原始数据为100Hz，那么 gnss_intervel=2 则降频为50Hz
opt.inital_yaw = 170;       % 初始方位角 deg (北偏东为正)

% 初始状态方差:    水平姿态           航向       东北天速度      水平位置   高度      陀螺零偏                 加速度计零偏
opt.P0 = diag([(2*rad)*ones(1,2), (180*rad), 0.5*ones(1,3), 5*ones(1,2), 10, (50/3600*rad)*ones(1,3), (10e-3*g)*ones(1,3)])^2;
% 系统方差:       角度随机游走           速度随机游走
opt.Q = diag([(1/60*rad)*ones(1,3),  (2/60)*ones(1,3),  zeros(1,9)])^2;

%% 数据载入
load('data20220303.mat');
imu_data = imu_data(1: opt.imu_intervel: end, :);
gnss_data = gnss_data(1: opt.gnss_intervel: end, :);
imu_length = length(imu_data);
gnss_length = length(gnss_data);

imu_time = imu_data(:, 1) / 1000;
gyro_data = imu_data(:, 5:7);
acc_data = imu_data(:, 2:4);

gnss_time = gnss_data(:, 1) / 1000;
lla_data = gnss_data(:, 4:6);
vel_data = gnss_data(:, 7:9);
pos_std_data = gnss_data(:, 10:12);
vel_std_data = gnss_data(:, 13:15);

imu_dt = mean(diff(imu_time));
gnss_dt = mean(diff(gnss_time));
gyro_bias0 = mean(gyro_data(1:opt.alignment_time,:));

%% 经纬度转换为当地东北天坐标系
lat0 = lla_data(1, 1);
lon0 = lla_data(1, 2);
alt0 = lla_data(1, 3);

Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sind(lat0)*sind(lat0));
Rn = Re * (1 + Earth_e*sind(lat0)*sind(lat0));
Rmh = Rm + alt0;
Rnh = Rn + alt0;

distance_sum = 0;
time_sum = 0;
gnss_enu = zeros(gnss_length, 3);
log.vel_norm = zeros(gnss_length, 1);
for i=1:gnss_length
    gnss_enu(i,3) = lla_data(i,3) - alt0;
    gnss_enu(i,2) = (lla_data(i,1) - lat0) * rad * (Rmh);
    gnss_enu(i,1) = (lla_data(i,2) - lon0) * rad * (Rnh) * cosd(lat0);
    
    log.vel_norm(i) = norm(vel_data(i, :));
    distance_sum = distance_sum + log.vel_norm(i)*gnss_dt;
    time_sum = time_sum + gnss_dt;
end

% plot_enu_vel(gnss_enu, log.vel_norm);

%% 初始参数设置
% 粗对准
g_b = - mean(acc_data(1:opt.alignment_time, :))';
g_b = g_b/norm(g_b);
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw*rad;
nQb = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
nQb_sins = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
std_acc_sldwin = 100; % acc 滑窗标准差
std_gyr_sldwin = 100; % gyr 滑窗标准差
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(15,1);
X_temp = X;
gyro_bias = X(10:12);
acc_bias = X(13:15);

P = opt.P0;
Q = opt.Q * imu_dt;

log.att = zeros(imu_length, 3);
log.vel = zeros(imu_length, 3);
log.pos = zeros(imu_length, 3);
log.P = zeros(imu_length, 15);
log.X = zeros(imu_length, 15);
if opt.sins_enable
    log.sins_att = zeros(imu_length, 3);
end

gnss_index = 1;

tic;
count_sec = 1;
for i=1:imu_length
    current_time = toc;
    if current_time>count_sec
        percent = (i)/(imu_length);
        clc;
        fprintf('已处理%.3f%%, 用时%.3f秒, 预计还需%.3f秒\n', (i)/(imu_length)*100, current_time, current_time/percent*(1-percent));
        count_sec = count_sec + 1;
    end
    
    %% 捷联更新
    % 单子样等效旋转矢量法
    w_b = gyro_data(i,:)'*rad - gyro_bias;
    f_b = acc_data(i,:)'*g - acc_bias;
    
    % 纯惯性姿态更新
    [nQb, pos, vel, q] = ins(w_b, f_b, nQb, pos, vel, g, imu_dt);
    
    if opt.sins_enable
        nQb_sins = quatmultiply(nQb_sins, q); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
        nQb_sins = quatnormalize(nQb_sins); %单位化四元数
    end
    
    bQn = quatinv(nQb); %更新bQn
    f_n = quatrotate(bQn, f_b')';
    nCb = quat2dcm(nQb); %更新nCb阵
    bCn = nCb'; %更新bCn阵
    
    %% 卡尔曼滤波
    F = zeros(15);
    F(4,2) = -f_n(3); %f_u天向比力
    F(4,3) = f_n(2); %f_n北向比力
    F(5,1) = f_n(3); %f_u天向比力
    F(5,3) = -f_n(1); %f_e东向比力
    F(6,1) = -f_n(2); %f_n北向比力
    F(6,2) = f_n(1); %f_e东向比力
    F(7:9,4:6) = eye(3);
    F(1:3,10:12) = -bCn;
    F(4:6,13:15) = bCn;
    
    % 状态转移矩阵F离散化
    F = eye(15) + F*imu_dt;
    
    % 卡尔曼时间更新
    X = F*X;
    P = F*P*F' + Q;
    
    %% 建立acc ,gyr 滑窗 并求基本统计量
    if(i > 5)
        std_acc_sldwin = sum(std(acc_data(i-5:i, :)));
        std_gyr_sldwin = sum(std(gyro_data(i-5:i, :)));
        
        log.std_acc_sldwin(i,1) = std_acc_sldwin;
        log.std_gyr_sldwin(i,1) = std_gyr_sldwin;
    end
    %% 重力量测
    if opt.gravity_update_enable && abs(norm(f_n)-9.8)<0.05 && (std_gyr_sldwin < 0.2) && (std_acc_sldwin < 0.1)
        H = zeros(2,15);
        H(1, 2) = 1;
        H(2, 1) = -1;
        g_n = -f_n/norm(f_n);
        
        Z = g_n - [0;0;-1];
        Z = Z(1:2);
        
        R = diag([100 100]);
        
        % 卡尔曼量测更新
        K = P * H' / (H * P * H' + R);
        X = X + K * (Z - H * X);
        P = (eye(length(X)) - K * H) * P;
        
        % 姿态修正
        rv = X(1:3);
        rv_norm = norm(rv);
        if rv_norm ~= 0
            qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
            nQb = quatmultiply(qe, nQb);
            nQb = quatnormalize(nQb); %单位化四元数
            bQn = quatinv(nQb); %更新bQn
            nCb = quat2dcm(nQb); %更新nCb阵
            bCn = nCb'; %更新bCn阵
        end
        
        % 暂存状态X
        X_temp = X;
        
        % 误差清零
        X(1:3) = zeros(3,1);
        
        % 零偏反馈
        if opt.bias_feedback
            gyro_bias = X(10:12);
            acc_bias = X(13:15);
            %         X(10:12) = zeros(3,1);
            %         X(13:15) = zeros(3,1);
        end
    end
    
    %% GNSS量测更新
    if (abs(imu_time(i) - gnss_time(gnss_index)) < imu_dt)
        if( ~opt.gnss_outage || imu_time(i) < opt.outage_start || imu_time(i) > opt.outage_stop )
            H = zeros(6,15);
            H(1:3,4:6) = eye(3);
            H(4:6,7:9) = eye(3);
            
            Z = [vel - vel_data(gnss_index,:)'; pos - gnss_enu(gnss_index,:)'];
            
            R = diag([0.2*ones(2,1); 0.2; 1*ones(2,1); 2])^2;
            %             R = diag([vel_std_data(gnss_index,:)  pos_std_data(gnss_index,:)])^2;
            
            % 卡尔曼量测更新
            K = P * H' / (H * P * H' + R);
            X = X + K * (Z - H * X);
            P = (eye(length(X)) - K * H) * P;
            
            % 姿态修正
            rv = X(1:3);
            rv_norm = norm(rv);
            if rv_norm ~= 0
                qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
                nQb = quatmultiply(qe, nQb);
                nQb = quatnormalize(nQb); %单位化四元数
                bQn = quatinv(nQb); %更新bQn
                nCb = quat2dcm(nQb); %更新nCb阵
                bCn = nCb'; %更新bCn阵
            end
            
            % 速度修正
            vel = vel - X(4:6);
            
            % 位置修正
            pos = pos - X(7:9);
            
            % 暂存状态X
            X_temp = X;
            
            % 误差清零
            X(1:9) = zeros(9,1);
            
            % 零偏反馈
            if opt.bias_feedback
                gyro_bias = X(10:12);
                acc_bias = X(13:15);
                %         X(10:12) = zeros(3,1);
                %         X(13:15) = zeros(3,1);
            end
        end
        
        % gnss_index到下一个GNSS数据点
        gnss_index = min(gnss_index+1, length(gnss_time));
    end
    
    % 信息存储
    [yaw, pitch, roll] = quat2angle(nQb, 'ZXY');
    yaw = -yaw;
    yaw = yaw + (yaw<0)*2*pi;
    log.att(i,:) = [pitch roll yaw]*deg;
    log.vel(i,:) = vel';
    log.pos(i,:) = pos';
    log.X(i, :) = X_temp';
    log.P(i, :) = sqrt(diag(P))';
    
    % 纯惯性信息存储
    if opt.sins_enable
        [yaw, pitch, roll] = quat2angle(nQb_sins, 'ZXY');
        yaw = -yaw;
        yaw = yaw + (yaw<0)*2*pi;
        log.sins_att(i,:) = [pitch roll yaw]*deg;
    end
end
clc;
fprintf('数据处理完毕，用时%.3f秒\n', toc);

%% 当地东北天坐标系转换成经纬度
kf_lla = zeros(imu_length, 3);
for i=1:imu_length
    kf_lla(i,3) = log.pos(i,3) + alt0;
    kf_lla(i,1) = log.pos(i,2) / Rmh * deg + lat0;
    kf_lla(i,2) = log.pos(i,1) / Rnh / cosd(lat0) * deg + lon0;
end

%% Google Map
% wm = webmap('World Imagery');
% wmline(kf_lla(:,1), kf_lla(:,2), 'Color', 'blue', 'Width', 1, 'OverlayName', 'KF');
% wmline(lla_data(:,1), lla_data(:,2), 'Color', 'red', 'Width', 1, 'OverlayName', 'GNSS');
% 线性可选颜色：
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'

%% 二维轨迹对比
figure('name', '二维轨迹对比');
plot(log.pos(:,1), log.pos(:,2), 'b'); hold on;
plot(gnss_enu(:,1), gnss_enu(:,2), 'r');
plot(log.pos(:,1), log.pos(:,2), 'b.');
plot(gnss_enu(:,1), gnss_enu(:,2), 'r.');
axis equal; grid on;
legend('KF', 'GNSS');
xlabel('East(m)');
ylabel('North(m)');
title('二维轨迹对比');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 姿态与航向估计曲线
figure('name', '姿态与航向估计曲线');
subplot(2,1,1);
plot((1:imu_length)/100, log.att(:,1), 'linewidth', 1.5); hold on; grid on;
plot((1:imu_length)/100, log.att(:,2), 'linewidth', 1.5);
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('水平姿态(°)'); legend('Pitch', 'Roll');
subplot(2,1,2);
plot((1:imu_length)/100, log.att(:,3), 'linewidth', 1.5); hold on; grid on;
if opt.sins_enable
    plot((1:imu_length)/100, log.sins_att(:,3), 'linewidth', 1.5);
    legend('Yaw', '纯惯性', 'Orientation','horizontal');
end
xlim([0 imu_length/100]);
ylim([-30 420]);
xlabel('时间(s)'); ylabel('航向(°)');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 速度估计曲线对比
figure('name', '速度估计曲线对比');
subplot(3,1,1);
plot((gnss_time-imu_time(1)), vel_data(:,1), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.vel(:,1), 'b');
plot((gnss_time-imu_time(1)), vel_data(:,1), 'r.');
plot((imu_time-imu_time(1)), log.vel(:,1), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('东向速度(m/s)'); legend('KF', 'GNSS');
subplot(3,1,2);
plot((gnss_time-imu_time(1)), vel_data(:,2), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.vel(:,2), 'b');
plot((gnss_time-imu_time(1)), vel_data(:,2), 'r.');
plot((imu_time-imu_time(1)), log.vel(:,2), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('北向速度(m/s)'); legend('KF', 'GNSS');
subplot(3,1,3);
plot((gnss_time-imu_time(1)), vel_data(:,3), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.vel(:,3), 'b');
plot((gnss_time-imu_time(1)), vel_data(:,3), 'r.');
plot((imu_time-imu_time(1)), log.vel(:,3), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('天向速度(m/s)'); legend('KF', 'GNSS');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 位置估计曲线对比
figure('name', '位置估计曲线对比');
subplot(3,1,1);
plot((gnss_time-imu_time(1)), gnss_enu(:,1), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.pos(:,1), 'b');
plot((gnss_time-imu_time(1)), gnss_enu(:,1), 'r.');
plot((imu_time-imu_time(1)), log.pos(:,1), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('东向位置(m)'); legend('KF', 'GNSS');
subplot(3,1,2);
plot((gnss_time-imu_time(1)), gnss_enu(:,2), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.pos(:,2), 'b');
plot((gnss_time-imu_time(1)), gnss_enu(:,2), 'r.');
plot((imu_time-imu_time(1)), log.pos(:,2), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('北向位置(m)'); legend('KF', 'GNSS');
subplot(3,1,3);
plot((gnss_time-imu_time(1)), gnss_enu(:,3), 'r'); hold on; grid on;
plot((imu_time-imu_time(1)), log.pos(:,3), 'b');
plot((gnss_time-imu_time(1)), gnss_enu(:,3), 'r.');
plot((imu_time-imu_time(1)), log.pos(:,3), 'b.');
xlim([0 max((imu_time-imu_time(1)))]);
xlabel('时间(s)'); ylabel('天向位置(m)'); legend('KF', 'GNSS');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% IMU零偏估计曲线
figure('name', 'IMU零偏估计曲线');
subplot(2,2,1);
plot((1:imu_length)/100, log.X(:, 10) * 3600 * deg, 'r', 'linewidth', 1.5); hold on; grid on;
plot((1:imu_length)/100, log.X(:, 11) * 3600 * deg, 'g', 'linewidth', 1.5);
plot((1:imu_length)/100, log.X(:, 12) * 3600 * deg, 'b', 'linewidth', 1.5);
plot((1:imu_length)/100, gyro_bias0(1) * 3600 * ones(imu_length,1), 'r-.', 'linewidth', 1);
plot((1:imu_length)/100, gyro_bias0(2) * 3600 * ones(imu_length,1), 'g-.', 'linewidth', 1);
plot((1:imu_length)/100, gyro_bias0(3) * 3600 * ones(imu_length,1), 'b-.', 'linewidth', 1);
xlim([0 imu_length/100]);
title('陀螺仪零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(°/h)'); legend('X', 'Y', 'Z');
subplot(2,2,3);
plot((1:imu_length)/100, log.P(:, 10:12) * 3600 * deg, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
title('陀螺仪零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(°/h)'); legend('X', 'Y', 'Z');

subplot(2,2,2);
plot((1:imu_length)/100, log.X(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
title('加速度计零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(mg)'); legend('X', 'Y', 'Z');
subplot(2,2,4);
plot((1:imu_length)/100, log.P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
title('加速度计零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(mg)'); legend('X', 'Y', 'Z');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 状态量曲线
figure('name','状态量曲线');
subplot(2,2,1);
plot((1:imu_length)/100, log.X(:, 1:2) * deg, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Orientation','horizontal');

subplot(2,2,3);
plot((1:imu_length)/100, log.X(:, 3) * deg, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Yaw', 'Orientation','horizontal');

subplot(2,2,2);
plot((1:imu_length)/100, log.X(:, 4:6), 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('东', '北', '天', 'Orientation','horizontal');

subplot(2,2,4);
plot((1:imu_length)/100, log.X(:, 7:9), 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('东', '北', '天', 'Orientation','horizontal');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% P阵收敛结果
figure('name','P阵收敛结果');
subplot(3,2,1);
plot((1:imu_length)/100, log.P(:, 1:3) * deg, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Yaw');
subplot(3,2,3);
plot((1:imu_length)/100, log.P(:, 4:6), 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('Ve', 'Vn', 'Vu');
subplot(3,2,5);
plot((1:imu_length)/100, log.P(:, 7:9), 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('Lat', 'Lon', 'Alt');
subplot(3,2,2);
plot((1:imu_length)/100, log.P(:, 10:12) * 3600 * deg, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('陀螺零偏(°/h)'); legend('X', 'Y', 'Z');
subplot(3,2,4);
plot((1:imu_length)/100, log.P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([0 imu_length/100]);
xlabel('时间(s)'); ylabel('加速度计零偏(mg)'); legend('X', 'Y', 'Z');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 数据统计
fprintf("IMU起始时间:%.3fs, GNSS起始时间:%.3fs\n", imu_time(1), gnss_time(1));
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('行驶时间: %d小时%d分%.3f秒\n', degrees2dms(time_sum/3600));
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);
fprintf('平均时速: %.3fkm/h\n', distance_sum/time_sum*3.6);
