close all;
clear;
clc;

format long g;
format compact;

R2D = 180/pi;
D2R = pi/180;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;

%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)

%% 相关选项及参数设置
opt.alignment_time = 1e2;  % 初始对准时间
opt.bias_feedback = true;  % IMU零偏反馈
opt.gnss_outage = false;    % 模拟GNSS丢失
opt.gravity_update_enable = false; % 使能重力静止量更新
opt.zupt_enable = false;     % ZUPT
opt.outage_start = 500;     % 丢失开始时间
opt.outage_stop = 560;      % 丢失结束时间
opt.gnss_intervel = 1;      % GNSS间隔时间，如原始数据为10Hz，那么 gnss_intervel=10 则降频为1Hz
opt.imu_intervel = 1;       % IMU间隔时间，如原始数据为100Hz，那么 gnss_intervel=2 则降频为50Hz
opt.inital_yaw = 90;       % 初始方位角 deg (北偏东为正)

% 初始状态方差:    水平姿态           航向       东北天速度      水平位置   高度      陀螺零偏                 加速度计零偏
opt.P0 = diag([(2*D2R)*ones(1,2), (180*D2R), 0.5*ones(1,2), 1, 5*ones(1,2), 10, (50/3600*D2R)*ones(1,3), (10e-3*g)*ones(1,3)])^2;
% 系统方差:       角度随机游走           速度随机游走
opt.Q = diag([(1/60*D2R)*ones(1,3), (2/60)*ones(1,3), 0*ones(1,3), 0*ones(1,3), 0*ones(1,3)])^2;

%% 数据载入
% load('data20220320_31.mat');
% load('data20220320_32.mat');

% load('data20220327_rtk1hz_1.mat');
% opt.inital_yaw = 90;

% load('data20220327_rtk1hz_2.mat');
% opt.inital_yaw = 270;

% load('data20220405_Standalone.mat');
load('data20220405_RTK.mat');
opt.inital_yaw = 90;

RTK_index = find(gnss_data(:,3)==4 | gnss_data(:,3)==5);
gnss_data(RTK_index, 8:10) = gnss_data(RTK_index-1, 8:10);

% gnss_data(find(gnss_data(:,3)==1),:)=[];
gnss_data(find(gnss_data(:,3)==4),:)=[];
gnss_data(find(gnss_data(:,3)==5),:)=[];

imu_data = imu_data(1: opt.imu_intervel: end, :);
gnss_data = gnss_data(1: opt.gnss_intervel: end, :);
imu_length = length(imu_data);
gnss_length = length(gnss_data);

imu_time = (imu_data(:, 2) - imu_data(1, 2));
gyro_data = imu_data(:, 6:8);
acc_data = imu_data(:, 3:5);

gnss_time = (gnss_data(:, 2) - imu_data(1, 2));
lla_data = gnss_data(:, 5:7);
vel_data = gnss_data(:, 8:10);

% lat0 = lla_data(1, 1);
% lon0 = lla_data(1, 2);
% alt0 = lla_data(1, 3);
% [R_meridian, R_transverse, C_ECEF2ENU, C_ECEF2NED] = ch_earth(lat0, lon0, alt0);
% for i=1:gnss_length
%     vel_data(i,:) = C_ECEF2ENU*vel_data(i, 1:3)';
% end

pos_std_data = gnss_data(:, 11:13);
vel_std_data = gnss_data(:, 14:16);

span_time = (span.time - imu_data(1, 2));
span_length = length(span.time);
span.lla = [span.lat span.lon span.alt];

imu_dt = mean(diff(imu_time));
gnss_dt = mean(diff(gnss_time));
gyro_bias0 = mean(gyro_data(1:opt.alignment_time,:));

%% 经纬度转换为当地东北天坐标系
lat0 = lla_data(1, 1);
lon0 = lla_data(1, 2);
alt0 = lla_data(1, 3);

Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
Rn = Re * (1 + Earth_e*sin(lat0)*sin(lat0));
Rmh = Rm + alt0;
Rnh = Rn + alt0;

distance_sum = 0;
time_sum = 0;
gnss_enu = zeros(gnss_length, 3);
log.vel_norm = zeros(gnss_length, 1);
for i=1:gnss_length
    gnss_enu(i,3) = lla_data(i,3) - alt0;
    gnss_enu(i,2) = (lla_data(i,1) - lat0) * (Rmh);
    gnss_enu(i,1) = (lla_data(i,2) - lon0) * (Rnh) * cos(lat0);

    log.vel_norm(i) = norm(vel_data(i, :));
    distance_sum = distance_sum + log.vel_norm(i)*gnss_dt;
    time_sum = time_sum + gnss_dt;
end

% plot_enu_vel(gnss_enu, log.vel_norm);

%% MCU结果转换为当地东北天坐标系
span_enu = zeros(span_length, 3);
span_enu(:,3) = span.lla(:,3) - alt0;
span_enu(:,2) = (span.lla(:,1) * D2R - lat0) * (Rmh);
span_enu(:,1) = (span.lla(:,2) * D2R - lon0) * (Rnh) * cos(lat0);
%% 初始参数设置
% 粗对准
g_b = - mean(acc_data(1:opt.alignment_time, :))';
g_b = g_b/norm(g_b);
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw*D2R;
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
log.gyro_bias = zeros(imu_length, 3);
log.acc_bias = zeros(imu_length, 3);
log.sins_att = zeros(imu_length, 3);
log.zupt_time = zeros(imu_length, 1);

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
    w_b = gyro_data(i,:)' - gyro_bias;
    f_b = acc_data(i,:)' - acc_bias;
    
    % 纯惯性姿态更新
    [nQb, pos, vel, q] = ins(w_b, f_b, nQb, pos, vel, g, imu_dt);
    
    nQb_sins = ch_qmul(nQb_sins, q); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
    nQb_sins = ch_qnormlz(nQb_sins); %单位化四元数
    
    bQn = ch_qconj(nQb); %更新bQn
    f_n = ch_qmulv(nQb, f_b);
    bCn = ch_q2m(nQb); %更新bCn阵
    nCb = bCn'; %更新nCb阵
    
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
    
    %% 建立IMU滑窗,并求基本统计量
    if(i > 50)
        std_acc_sldwin = sum(std(acc_data(i-50:i, :)));
        std_gyr_sldwin = sum(std(gyro_data(i-50:i, :)));
        
        log.std_acc_sldwin(i,1) = std_acc_sldwin;
        log.std_gyr_sldwin(i,1) = std_gyr_sldwin;
    end
    
    %% 静止条件判断
    if abs(norm(f_n)-9.8)<0.3 && (std_gyr_sldwin < 0.3) && (std_acc_sldwin < 0.01)
       log.zupt_time(i) = 1;
       
       %% ZUPT
       if opt.zupt_enable
           H = zeros(3, 15);
           H(1:3,4:6) = eye(3);

           Z = vel;

           R = diag(0.2*ones(1,3))^2;
           
           K = P * H' / (H * P * H' + R);
           X = X + K * (Z - H * X);
           P = (eye(length(X)) - K * H) * P;

           % 姿态修正
           rv = X(1:3);
           rv_norm = norm(rv);
           if rv_norm ~= 0
               qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
               nQb = ch_qmul(qe, nQb);
               nQb = ch_qnormlz(nQb); %单位化四元数
               bQn = ch_qconj(nQb); %更新bQn
               bCn = ch_q2m(nQb); %更新bCn阵
               nCb = bCn'; %更新nCb阵
           end

           % 速度修正
           vel = vel - X(4:6);

           % 暂存状态X
           X_temp = X;

           % 误差清零
           X(1:6) = zeros(6,1);

           % 零偏反馈
           if opt.bias_feedback
               gyro_bias = X(10:12);
               acc_bias = X(13:15);
           end
       end

        %% 静止状态下重力量测更新姿态
        if opt.gravity_update_enable
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
                nQb = ch_qmul(qe, nQb);
                nQb = ch_qnormlz(nQb); %单位化四元数
                bQn = ch_qconj(nQb); %更新bQn
                bCn = ch_q2m(nQb); %更新bCn阵
                nCb = bCn'; %更新nCb阵
            end
            
            % 暂存状态X
            X_temp = X;
            
            % 误差清零
            X(1:3) = zeros(3,1);
            
            % 零偏反馈
            if opt.bias_feedback
                gyro_bias = X(10:12);
                acc_bias = X(13:15);
            end
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

            % 只进行GNSS位置修正
%             H = zeros(3,15);
%             H(1:3,7:9) = eye(3);
%             Z = [pos - gnss_enu(gnss_index,:)'];
%             R = diag([1*ones(2,1); 2])^2;
            
            % 卡尔曼量测更新
            K = P * H' / (H * P * H' + R);
            X = X + K * (Z - H * X);
            P = (eye(length(X)) - K * H) * P;
            
            % 姿态修正
            rv = X(1:3);
            rv_norm = norm(rv);
            if rv_norm ~= 0
                qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
                nQb = ch_qmul(qe, nQb);
                nQb = ch_qnormlz(nQb); %单位化四元数
                bQn = ch_qconj(nQb); %更新bQn
                bCn = ch_q2m(nQb); %更新bCn阵
                nCb = bCn'; %更新nCb阵
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
                gyro_bias = gyro_bias + X(10:12);
                acc_bias = acc_bias + X(13:15);
                X(10:12) = zeros(3,1);
                X(13:15) = zeros(3,1);
            end
        end
        
        % gnss_index到下一个GNSS数据点
        gnss_index = min(gnss_index+1, length(gnss_time));
    end
    
    % 信息存储
    [pitch,roll,yaw] = q2att(nQb);
    log.att(i,:) = [pitch roll yaw];
    log.vel(i,:) = vel';
    log.pos(i,:) = pos';
    log.X(i, :) = X_temp';
    log.P(i, :) = sqrt(diag(P))';
    log.gyro_bias(i, :) = gyro_bias;
    log.acc_bias(i, :) = acc_bias;
    
    % 纯惯性信息存储
    [pitch,roll,yaw] = q2att(nQb_sins);
    log.sins_att(i,:) = [pitch roll yaw];
end
clc;
fprintf('数据处理完毕，用时%.3f秒\n', toc);

%% 当地东北天坐标系转换成经纬度
kf_lla = zeros(imu_length, 3);
for i=1:imu_length
    kf_lla(i,3) = log.pos(i,3) + alt0;
    kf_lla(i,1) = log.pos(i,2) / Rmh + lat0;
    kf_lla(i,2) = log.pos(i,1) / Rnh / cos(lat0) + lon0;
end

%% Google Map
% wm = webmap('World Imagery');
% wmline(kf_lla(:,1)*R2D, kf_lla(:,2)*R2D, 'Color', 'blue', 'Width', 1, 'OverlayName', 'KF');
% wmline(lla_data(:,1)*R2D, lla_data(:,2)*R2D, 'Color', 'red', 'Width', 1, 'OverlayName', 'GNSS');
% wmline(span.lla(:,1), span.lla(:,2), 'Color', 'red', 'Width', 1, 'OverlayName', 'MCU');
% 线性可选颜色：
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'

%% 姿态与航向估计曲线
figure('name', '姿态与航向估计曲线');
subplot(3,1,1);
plot(imu_time, log.att(:,1), 'linewidth', 1.5); hold on; grid on;
plot(span_time, span.att(:,1), 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylabel('Pitch(°)'); legend('MATLAB', 'MCU', 'Orientation','horizontal');
subplot(3,1,2);
plot(imu_time, log.att(:,2), 'linewidth', 1.5);  hold on; grid on;
plot(span_time, span.att(:,2), 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylabel('Roll(°)'); legend('MATLAB', 'MCU', 'Orientation','horizontal');
subplot(3,1,3);
plot(imu_time, log.att(:,3), 'linewidth', 1.5); hold on; grid on;
plot(span_time, span.att(:,3), 'linewidth', 1.5);
plot(imu_time, log.sins_att(:,3), 'linewidth', 1.5);
legend('MATLAB', 'MCU', '纯惯性', 'Orientation','horizontal');
xlim([imu_time(1) imu_time(end)]);
ylim([-30 420]);
xlabel('时间(s)'); ylabel('Yaw(°)');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 速度估计曲线对比
figure('name', '速度估计曲线对比');
subplot(3,1,1);
plot(gnss_time, vel_data(:,1), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.vel(:,1), 'b', 'Marker','.');
plot(span_time, span.vel(:,1), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
ylabel('东向速度(m/s)'); legend('GNSS', 'MATLAB', 'MCU');
subplot(3,1,2);
plot(gnss_time, vel_data(:,2), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.vel(:,2), 'b', 'Marker','.');
plot(span_time, span.vel(:,2), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
ylabel('北向速度(m/s)'); legend('GNSS', 'MATLAB', 'MCU');
subplot(3,1,3);
plot(gnss_time, vel_data(:,3), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.vel(:,3), 'b', 'Marker','.');
plot(span_time, span.vel(:,3), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('天向速度(m/s)'); legend('GNSS', 'MATLAB', 'MCU');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 位置估计曲线对比
figure('name', '位置估计曲线对比');
subplot(3,1,1);
plot(gnss_time, gnss_enu(:,1), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.pos(:,1), 'b', 'Marker','.');
plot(span_time, span_enu(:,1), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
ylabel('东向位置(m)'); legend('GNSS', 'MATLAB', 'MCU');
subplot(3,1,2);
plot(gnss_time, gnss_enu(:,2), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.pos(:,2), 'b', 'Marker','.');
plot(span_time, span_enu(:,2), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
ylabel('北向位置(m)'); legend('GNSS', 'MATLAB', 'MCU');
subplot(3,1,3);
plot(gnss_time, gnss_enu(:,3), 'r', 'Marker','.'); hold on; grid on;
plot(imu_time, log.pos(:,3), 'b', 'Marker','.');
plot(span_time, span_enu(:,3), 'm', 'Marker','.');
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('天向位置(m)'); legend('GNSS', 'MATLAB', 'MCU');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% IMU零偏估计曲线
figure('name', 'IMU零偏估计曲线');
subplot(2,2,1);
color_rgb = colororder;
plot(imu_time, log.gyro_bias(:, 1) * 3600 * R2D, 'Color', color_rgb(1,:), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.gyro_bias(:, 2) * 3600 * R2D, 'Color', color_rgb(2,:), 'linewidth', 1.5);
plot(imu_time, log.gyro_bias(:, 3) * 3600 * R2D, 'Color', color_rgb(3,:), 'linewidth', 1.5);
plot(imu_time, gyro_bias0(1) * 3600 * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(1,:), 'linewidth', 1);
plot(imu_time, gyro_bias0(2) * 3600 * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(2,:), 'linewidth', 1);
plot(imu_time, gyro_bias0(3) * 3600 * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(3,:), 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
title('陀螺仪零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(°/h)'); legend('X', 'Y', 'Z');

subplot(2,2,2);
plot(imu_time, log.acc_bias(:, 1:3) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
title('加速度计零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(mg)'); legend('X', 'Y', 'Z');

subplot(2,2,3);
semilogy(imu_time, log.P(:, 10:12) * 3600 * R2D, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
title('陀螺仪零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(°/h)'); legend('X', 'Y', 'Z');

subplot(2,2,4);
semilogy(imu_time, log.P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
title('加速度计零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(mg)'); legend('X', 'Y', 'Z');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 状态量曲线
figure('name','状态量曲线');
subplot(2,2,1);
plot(imu_time, log.X(:, 1) * R2D, 'c', 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.X(:, 2) * R2D, 'm', 'linewidth', 1.5);
plot(imu_time, log.P(:, 1) * R2D * 3, 'r-.', 'linewidth', 1);
plot(imu_time, log.P(:, 2) * R2D * 3, 'g-.', 'linewidth', 1);
plot(imu_time, log.P(:, 1) * R2D * -3, 'r-.', 'linewidth', 1);
plot(imu_time, log.P(:, 2) * R2D * -3, 'g-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-0.5 0.5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Orientation','horizontal');

subplot(2,2,3);
plot(imu_time, log.X(:, 3) * R2D, 'c', 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 3) * R2D * 3, 'b-.', 'linewidth', 1);
plot(imu_time, log.P(:, 3) * R2D * -3, 'b-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-5 5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Yaw', 'Orientation','horizontal');

subplot(2,2,2);
plot(imu_time, log.X(:, 4:6), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 4:6) * R2D * 3, '-.', 'linewidth', 1);
plot(imu_time, log.P(:, 4:6) * R2D * -3, '-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-10 10]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('东', '北', '天', 'Orientation','horizontal');

subplot(2,2,4);
plot(imu_time, log.X(:, 7:9), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 7:9) * R2D * 3, '-.', 'linewidth', 1);
plot(imu_time, log.P(:, 7:9) * R2D * -3, '-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-100 100]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('东', '北', '天', 'Orientation','horizontal');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% P阵收敛结果
figure('name','P阵收敛结果');
subplot(3,2,1);
semilogy(imu_time, log.P(:, 1:3) * R2D, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Yaw');
subplot(3,2,2);
semilogy(imu_time, log.P(:, 4:6), 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('Ve', 'Vn', 'Vu');
subplot(3,2,4);
semilogy(imu_time, log.P(:, 7:9), 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('Lat', 'Lon', 'Alt');
subplot(3,2,3);
semilogy(imu_time, log.P(:, 10:12) * 3600 * R2D, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('陀螺零偏(°/h)'); legend('X', 'Y', 'Z');
subplot(3,2,5);
semilogy(imu_time, log.P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)'); ylabel('加速度计零偏(mg)'); legend('X', 'Y', 'Z');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 二维轨迹对比
figure('name', '二维轨迹对比');
plot(gnss_enu(:,1), gnss_enu(:,2), 'r', 'Marker','.'); hold on; axis equal; grid on;
plot(log.pos(:,1), log.pos(:,2), 'b', 'Marker','.');
plot(span_enu(:,1), span_enu(:,2), 'm', 'Marker','.');
legend('GNSS', 'MATLAB', 'MCU');
xlabel('East(m)');
ylabel('North(m)');
title('二维轨迹对比');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 静止检测验证
figure('name', '静止检测验证');
zupt_enable_index = find(log.zupt_time==1);
zupt_disable_index = find(log.zupt_time==0);
plot(zupt_enable_index*imu_dt, sum(abs(acc_data(zupt_enable_index,:)).^2,2).^(1/2), 'r.'); hold on; grid on;
plot(zupt_disable_index*imu_dt, sum(abs(acc_data(zupt_disable_index,:)).^2,2).^(1/2), 'b.');
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)');
ylabel('加速度模长(g)');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 数据统计
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('行驶时间: %d小时%d分%.3f秒\n', degrees2dms(time_sum/3600));
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);
fprintf('平均时速: %.3fkm/h\n', distance_sum/time_sum*3.6);
