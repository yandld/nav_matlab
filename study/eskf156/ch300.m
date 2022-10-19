close all;
clear;
clc;

format long g;
format compact;

N = 15;                 %ESKF维度
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;     % 重力加速度
Re = 6378137;      %地球半径
Earth_e = 0.00335281066474748; %地球扁率

%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)

%% 相关选项及参数设置
opt.alignment_time = 10e2;      % 初始对准时间
opt.bias_feedback = 1;          % IMU零偏反馈

opt.gravity_update_enable = 1;  % 使能重力静止量更新
opt.nhc_enable = 0;             % 车辆运动学约束

opt.zupt_acc_std = 0.2;        % 加速度计方差滑窗阈值
opt.zupt_gyr_std = 0.002;        % 陀螺仪方差滑窗阈值

opt.gnss_outage = 0;            % 模拟GNSS丢失
opt.outage_start = 2892;         % 丢失开始时间
opt.outage_stop = 3122;          % 丢失结束时间

opt.gnss_delay = 0;          % GNSS量测延迟 sec
opt.gravity_R = 0.2;          % 重力更新 噪声
opt.gnss_intervel = 1;          % GNSS间隔时间，如原始数据为10Hz，那么 gnss_intervel=10 则降频为1Hz

%状态量限幅
% opt.XMAX_PHI = 0.05*D2R*100000000;
% opt.XMAX_VEL = 10*100000000;
% opt.XMAX_POS = 100*100000000;
% opt.XMAX_GB = 1e-2 * D2R;
% opt.XMAX_WB = 1e-2 * GRAVITY;
% 
% opt.Xlimit_phi_xy = 0.1*D2R;
% opt.Xlimit_phi_z = 1*D2R;


% 初始状态方差:    姿态           东北天速度  水平位置      陀螺零偏                              加速度计零偏
opt.P0 = diag([ [2 2 5]*D2R, [0.1 0.1 0.1], [ 10 10 10],  [100 100 100]* D2R/3600, [10e-3, 10e-3, 10e-3]*GRAVITY])^2;
% 系统方差:       角度随机游走           速度随机游走                      角速度随机游走        加速度随机游走
opt.Q = diag([(0.5/60*D2R)*ones(1,3), (0.5/60)*ones(1,3), 0*ones(1,3), (1/3600*D2R)*ones(1,3), 1e-4*GRAVITY*ones(1,3)])^2;


%% 数据载入

% 
% load('dataset/2022年10月12日15时40分19秒.mat');
% opt.inital_yaw = 353;

% load('dataset/2022年10月17日15时30分23秒.mat');
% opt.inital_yaw = 90;


% load('dataset/2022年10月12日10时17分46秒.mat');
% opt.inital_yaw = 353;

% load('dataset/2022年10月12日15时40分19秒.mat');
% opt.inital_yaw = 87;

load('dataset/2022年10月12日16时38分35秒.mat');
opt.inital_yaw = 161;

% load('dataset/2022年10月17日13时37分50秒.mat');
% opt.inital_yaw = 161;


pos_type = data(:, 46);
evt_bit = data(:, 47);

imu_data = data(:, 21:26);
gnss_data = data(:, 27:44);
span_data = data(:, 3:20);
imu_length = length(imu_data);
gnss_length = length(gnss_data);
span_length = length(span_data);

imu_time = (data(:, 2) - data(1, 2));
span_time = imu_time;
gnss_time = imu_time;

gyro_data = imu_data(:, 1:3);
acc_data = imu_data(:, 4:6);

lla_data = gnss_data(:, [2 1 3]); lla_data(lla_data(:,1)==0, :) = NaN; %数据去0
lla_data(:,1:2) = lla_data(:,1:2)*D2R;
vel_data = gnss_data(:, 4:6);
bl_yaw = gnss_data(:, 7); bl_yaw(bl_yaw==0, :) = NaN; %数据去0
bl_pitch = gnss_data(:, 8); bl_pitch(bl_pitch==0, :) = NaN; %数据去0
bl_length = gnss_data(:, 9);

pos_std_data = gnss_data(:, 10:12);
vel_std_data = gnss_data(:, 13:15);

span.lla = span_data(:, [8 7 9]);
span.vel = span_data(:, 4:6);
span.att = span_data(:, 1:3);

last_gnss_evt = 1;
imu_dt = mean(diff(data(:,2)));
gnss_dt = imu_dt;
gyro_bias0 = mean(gyro_data(1:opt.alignment_time,:));

fprintf("gyro起始时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyro_bias0(1)*R2D, gyro_bias0(2)*R2D, gyro_bias0(3)*R2D);
gyro_bias_end = mean(gyro_data(end-opt.alignment_time:end, :));
fprintf("gyro结束时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyro_bias_end(1)*R2D, gyro_bias_end(2)*R2D, gyro_bias_end(3)*R2D);

%% EVT Bit
evt_gnss_mask = bitshift(1,0);
evt_gpst_mask = bitshift(1,2);
evt_ins_mask = bitshift(1,4);

% evt_gnss = bitand(evt_bit, evt_gnss_mask);
evt_gnss = zeros(gnss_length, 1);
for i=2:gnss_length
    if (gnss_data(i, 1)~=gnss_data(i-1, 1)) || ...
            (gnss_data(i, 2)~=gnss_data(i-1, 2)) || ...
            (gnss_data(i, 3)~=gnss_data(i-1, 3)) || ...
            (gnss_data(i, 4)~=gnss_data(i-1, 4)) || ...
            (gnss_data(i, 5)~=gnss_data(i-1, 5)) || ...
            (gnss_data(i, 6)~=gnss_data(i-1, 6))
        evt_gnss(i) = 1;
    end
end
evt_gpst = bitand(evt_bit, evt_gpst_mask);
evt_ins = bitand(evt_bit, evt_ins_mask);

% GNSS数据抽样
gnss_resample_index = find(evt_gnss == evt_gnss_mask);
gnss_resample_index = gnss_resample_index(1:opt.gnss_intervel:end);
evt_gnss = zeros(gnss_length, 1);
evt_gnss(gnss_resample_index) = evt_gnss_mask;

%% 检测GNSS数据何时更新
% gnss_update = zeros(gnss_length, 1);
% for i=2:gnss_length
%     if (gnss_data(i, 1)~=gnss_data(i-1, 1)) || ...
%             (gnss_data(i, 2)~=gnss_data(i-1, 2)) || ...
%             (gnss_data(i, 3)~=gnss_data(i-1, 3)) || ...
%             (gnss_data(i, 4)~=gnss_data(i-1, 4)) || ...
%             (gnss_data(i, 5)~=gnss_data(i-1, 5)) || ...
%             (gnss_data(i, 6)~=gnss_data(i-1, 6))
%         gnss_update(i) = 1;
%     end
% end
% 
% % GNSS数据抽样
% gnss_resample_index = find(gnss_update == 1);
% gnss_resample_index = gnss_resample_index(1:opt.gnss_intervel:end);
% gnss_update = zeros(gnss_length, 1);
% gnss_update(gnss_resample_index) = 1;

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
pitch_sins = pitch0;
roll_sins = roll0;
yaw_sins = yaw0;
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
log.vb = zeros(imu_length, 3);
log.zupt_time = zeros(imu_length, 1);
log.std_acc_sldwin = NaN(imu_length, 1);;
log.std_gyr_sldwin = NaN(imu_length, 1);;
log.mean_acc_sldwin = NaN(imu_length, 1);;
log.mean_gyr_sldwin = NaN(imu_length, 1);;

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
    
    % 捷联更新
    [nQb, pos, vel, ~] = ins(w_b, f_b, nQb, pos, vel, GRAVITY, imu_dt);

    % 纯捷联姿态
    [nQb_sins, ~, ~, ~] = ins(w_b, f_b, nQb_sins, pos, vel, GRAVITY, imu_dt);
    
    bQn = ch_qconj(nQb); %更新bQn
    f_n = ch_qmulv(nQb, f_b);
    a_n = f_n + [0; 0; -GRAVITY];
    bCn = ch_q2m(nQb); %更新bCn阵
    nCb = bCn'; %更新nCb阵

    log.vb(i, :) = (nCb * vel)';
    
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

        mean_acc_sldwin = sum(mean(acc_data(i-50:i, :)));
        mean_gyr_sldwin = sum(mean(gyro_data(i-50:i, :)));
        
        log.std_acc_sldwin(i) = std_acc_sldwin;
        log.std_gyr_sldwin(i) = std_gyr_sldwin;

        log.mean_acc_sldwin(i) = mean_acc_sldwin;
        log.mean_gyr_sldwin(i) = mean_gyr_sldwin;
    end
    

    %% 静止状态下重力量测更新姿态
    if opt.gravity_update_enable
        H = zeros(2,15);
        H(1, 2) = 1;
        H(2, 1) = -1;
        g_n = -f_n/norm(f_n);
        
        Z = g_n - [0;0;-1];
        Z = Z(1:2);
        
        R = diag([opt.gravity_R  opt.gravity_R])^2;
        
        % 卡尔曼量测更新
        K = P * H' / (H * P * H' + R);
        X = X + K * (Z - H * X);
        P = (eye(N) - K * H) * P;
        
        % 姿态修正
        rv = X(1:3);
        rv_norm = norm(rv);
        qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
        nQb = ch_qmul(qe, nQb);
        nQb = ch_qnormlz(nQb); %单位化四元数
        bQn = ch_qconj(nQb); %更新bQn
        bCn = ch_q2m(nQb); %更新bCn阵
        nCb = bCn'; %更新nCb阵
        X(1:3) = 0;
    end

        %% 静止条件判断
   % if (std_gyr_sldwin < opt.zupt_gyr_std) && (std_acc_sldwin < opt.zupt_acc_std)
 %   end
    %% GNSS量测更新
    if (evt_gnss(i) && ~isnan(gnss_enu(i,1)))
        last_gnss_evt = i;
        if( (~opt.gnss_outage || imu_time(i) < opt.outage_start || imu_time(i) > opt.outage_stop))
            if(norm(vel_std_data(i,:) - vel_std_data(i-1,:)) < 0.02) % 踢掉GNSS输出的可能的不可靠结果
            H = zeros(6,15);
            H(1:3,4:6) = eye(3);
            H(4:6,7:9) = eye(3);
            
            Z = [vel - vel_data(i,:)'; pos - gnss_enu(i,:)'];
            
            % GNSS量测延迟补偿
            Z = Z - [a_n; vel]*opt.gnss_delay;
            
            R = diag([vel_std_data(i,:)  pos_std_data(i,:)])^2;

            % 卡尔曼量测更新
            K = P * H' / (H * P * H' + R);
            X = X + K * (Z - H * X);
            P = (eye(N) - K * H) * P;
            
            %状态量限制

%             X(X(1:2) > opt.Xlimit_phi_xy) = opt.Xlimit_phi_xy;
%             X(X(1:2) < -opt.Xlimit_phi_xy) = -opt.Xlimit_phi_xy;
% 
%             X(X(3) > opt.Xlimit_phi_z) = opt.Xlimit_phi_z;
%             X(X(3) < -opt.Xlimit_phi_z) = -opt.Xlimit_phi_z;

            % 姿态修正
            rv = X(1:3);
            rv_norm = norm(rv);
                qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
                nQb = ch_qmul(qe, nQb);
                nQb = ch_qnormlz(nQb); %单位化四元数
                bQn = ch_qconj(nQb); %更新bQn
                bCn = ch_q2m(nQb); %更新bCn阵
                nCb = bCn'; %更新nCb阵
            
            vel = vel - X(4:6);
            pos = pos - X(7:9);
            X_temp = X;
            
            % 误差清零
            X(1:9) = 0;
            
            % 零偏反馈
            if opt.bias_feedback
                 gyro_bias = gyro_bias + X(10:12);
                 acc_bias = acc_bias + X(13:15);
                 X(10:12) = 0;
                 X(13:15) = 0;
            end
            end
        end
    end

      %  imu_time(i) > opt.outage_start && imu_time(i) < opt.outage_stop
      % (i - last_gnss_evt > 50)
      %1865
       if opt.nhc_enable && norm(gyro_data(i,:)) < 2*D2R && (i - last_gnss_evt > 50)
            H = zeros(3,15);
            H(1:3,4:6) = eye(3);

            vb_hmc = [log.vb(i,1); log.vb(i,2); log.vb(i,3)];
            vb_hmc(1) = 0;
            vb_hmc(3) = 0;
            vn_hmc = bCn * vb_hmc;
            
            Z = vel - vn_hmc;
            
            Rb = diag([ones(1,3)*1])^2;
            R = bCn*Rb*bCn';
           % R = Rb;
            
            % 卡尔曼量测更新
            K = P * H' / (H * P * H' + R);
            X = X + K * (Z - H * X);
            P = (eye(N) - K * H) * P;
            
       end

    
    %% 信息存储
    [pitch, roll, yaw] = q2att(nQb);
    log.att(i,:) = [pitch roll yaw];
    log.vel(i,:) = vel';
    log.pos(i,:) = pos';
    log.X(i, :) = X_temp';
    log.P(i, :) = sqrt(diag(P))';
    log.gyro_bias(i, :) = gyro_bias;
    log.acc_bias(i, :) = acc_bias;
    
    % 纯惯性信息存储
    [pitch_sins, roll_sins, yaw_sins] = q2att(nQb_sins);
    log.sins_att(i,:) = [pitch_sins roll_sins yaw_sins];
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

%% IMU原始数据
% plot_imu(gyro_data*R2D, acc_data, imu_dt);

%% 静止检测验证
figure('name', '静止检测验证');

subplot(3,2,1);
plot(imu_time, sum(acc_data.^2,2).^(1/2), 'linewidth', 0.1);
xlim([imu_time(1) imu_time(end)]);
ylim([9 11]);
ylabel('加速度瞬时模长(g)');

subplot(3,2,3);
plot(imu_time, log.std_acc_sldwin, 'linewidth', 1.5); hold on; grid on;
plot(imu_time, opt.zupt_acc_std*ones(size(imu_time)), '-.', 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylim([0 opt.zupt_acc_std*3]);
ylabel('加速度计方差滑窗(g)');

subplot(3,2,5);
plot(imu_time(2:end), diff(log.mean_acc_sldwin), 'linewidth', 1.5); hold on; grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)');
ylabel('加速度计均值滑窗差分(g)');

subplot(3,2,2);
plot(imu_time, sum(gyro_data.^2,2).^(1/2)*R2D, 'linewidth', 0.1);
xlim([imu_time(1) imu_time(end)]);
ylim([0 1]);
ylabel('陀螺仪瞬时模长(deg/s)');

subplot(3,2,4);
plot(imu_time, log.std_gyr_sldwin, 'linewidth', 1.5); hold on; grid on;
plot(imu_time, opt.zupt_gyr_std*ones(size(imu_time)), '-.', 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylim([0 opt.zupt_gyr_std*3]);
ylabel('陀螺仪方差滑窗(rad/s)');

subplot(3,2,6);
plot(imu_time(2:end), diff(log.mean_gyr_sldwin), 'linewidth', 1.5); hold on; grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)');
ylabel('陀螺仪均值滑窗差分(rad/s)');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 静止检测验证
figure('name', '静止检测验证');
subplot(3,1,1);
zupt_enable_index = find(log.zupt_time==1);
zupt_disable_index = find(log.zupt_time==0);
plot(zupt_enable_index*imu_dt, sum(abs(acc_data(zupt_enable_index,:)).^2,2).^(1/2), 'r.'); hold on; grid on;
plot(zupt_disable_index*imu_dt, sum(abs(acc_data(zupt_disable_index,:)).^2,2).^(1/2), 'b.');
xlim([imu_time(1) imu_time(end)]);
ylim([7 12]);
ylabel('加速度模长(g)');

subplot(3,1,2);
plot(imu_time, log.std_acc_sldwin, 'linewidth', 1.5); hold on; grid on;
plot(imu_time, opt.zupt_acc_std*ones(size(imu_time)), '-.', 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylim([0 opt.zupt_acc_std*3]);
ylabel('加速度计方差滑窗(g)');

subplot(3,1,3);
plot(imu_time, log.std_gyr_sldwin, 'linewidth', 1.5); hold on; grid on;
plot(imu_time, opt.zupt_gyr_std*ones(size(imu_time)), '-.', 'linewidth', 1.5);
xlim([imu_time(1) imu_time(end)]);
ylim([0 opt.zupt_gyr_std*3]);
xlabel('时间(s)');
ylabel('陀螺仪方差滑窗(rad)');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% Google Map
% plot_google_map(lla_data*R2D, kf_lla*R2D, span.lla);
% plot_google_map(lla_data*R2D, kf_lla*R2D);

%% 二维轨迹与速度
% plot_enu_vel(gnss_enu, vecnorm(vel_data, 2, 2));

%% 姿态与航向估计曲线
plot_att(imu_time,log.att, span_time,span.att, imu_time,log.sins_att, imu_time,[bl_pitch bl_yaw]);

%% 速度估计曲线
% plot_vel(gnss_time,vel_data, imu_time,log.vel);
plot_vel(gnss_time,vel_data, imu_time,log.vel, span_time,span.vel);

%% 位置估计曲线
% plot_enu(gnss_time,gnss_enu, imu_time,log.pos);
plot_enu(gnss_time,gnss_enu, imu_time,log.pos, span_time,span_enu);

%% 二维轨迹
% plot_enu_2d(gnss_enu, log.pos);
plot_enu_2d(gnss_enu, log.pos, span_enu);

%% IMU零偏估计曲线
figure('name', 'IMU零偏估计曲线');
subplot(2,2,1);
color_rgb = get(gca,'ColorOrder');
plot(imu_time, log.gyro_bias(:, 1)  * R2D, 'Color', color_rgb(1,:), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.gyro_bias(:, 2)  * R2D, 'Color', color_rgb(2,:), 'linewidth', 1.5);
plot(imu_time, log.gyro_bias(:, 3)  * R2D, 'Color', color_rgb(3,:), 'linewidth', 1.5);
plot(imu_time, gyro_bias0(1)  * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(1,:), 'linewidth', 1);
plot(imu_time, gyro_bias0(2)  * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(2,:), 'linewidth', 1);
plot(imu_time, gyro_bias0(3)  * R2D * ones(imu_length,1), '-.', 'Color', color_rgb(3,:), 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
title('陀螺仪零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(dps)'); legend('X', 'Y', 'Z');

subplot(2,2,2);
plot(imu_time, log.acc_bias(:, 1:3) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
title('加速度计零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(mg)'); legend('X', 'Y', 'Z');

subplot(2,2,3);
semilogy(imu_time, log.P(:, 10:12)  * R2D, 'linewidth', 1.5); grid on;
xlim([imu_time(1) imu_time(end)]);
title('陀螺仪零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(dps)'); legend('X', 'Y', 'Z');

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
plot(imu_time, log.P(:, 1) * R2D * 1, 'r-.', 'linewidth', 1);
plot(imu_time, log.P(:, 2) * R2D * 1, 'g-.', 'linewidth', 1);
plot(imu_time, log.P(:, 1) * R2D * -1, 'r-.', 'linewidth', 1);
plot(imu_time, log.P(:, 2) * R2D * -1, 'g-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-0.5 0.5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Orientation','horizontal');

subplot(2,2,3);
plot(imu_time, log.X(:, 3) * R2D, 'c', 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 3) * R2D * 1, 'b-.', 'linewidth', 1);
plot(imu_time, log.P(:, 3) * R2D * -1, 'b-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-5 5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Yaw', 'Orientation','horizontal');

subplot(2,2,2);
plot(imu_time, log.X(:, 4:6), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 4:6) * R2D * 1, '-.', 'linewidth', 1);
plot(imu_time, log.P(:, 4:6) * R2D * -1, '-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-10 10]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('东', '北', '天', 'Orientation','horizontal');

subplot(2,2,4);
plot(imu_time, log.X(:, 7:9), 'linewidth', 1.5); hold on; grid on;
plot(imu_time, log.P(:, 7:9) * R2D * 1, '-.', 'linewidth', 1);
plot(imu_time, log.P(:, 7:9) * R2D * -1, '-.', 'linewidth', 1);
xlim([imu_time(1) imu_time(end)]);
ylim([-100 100]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('东', '北', '天', 'Orientation','horizontal');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% P阵收敛结果
plot_P(imu_time, log.P);

%% 组合导航结果与GNSS原始数据之间误差
% figure('name', '组合导航结果与GNSS原始数据之间误差');
% xest = log.pos(:,1)';
% yest = log.pos(:,2)';
% zest = log.pos(:,3)';
% xgps = interp1(gnss_time, gnss_enu(:,1), imu_time, 'linear','extrap')';
% ygps = interp1(gnss_time, gnss_enu(:,2), imu_time, 'linear','extrap')';
% zgps = interp1(gnss_time, gnss_enu(:,3), imu_time, 'linear','extrap')';
% xerr = xest - xgps;
% yerr = yest - ygps;
% zerr = zest - zgps;
% herr = sqrt(xerr.^2+yerr.^2);
% positionerr_RMS = rms(herr);
% 
% subplot(2,1,1);
% plot(imu_time, herr, 'LineWidth',1.5); grid on;
% xlim([0 imu_time(end)]);
% ylabel('水平位置误差(m)');
% 
% subplot(2,1,2);
% plot(imu_time, zerr, 'LineWidth',1.5); grid on;
% xlim([0 imu_time(end)]);
% xlabel('时间(s)');
% ylabel('高度误差(m)');
% 
% set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 数据统计
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('行驶时间: %d小时%d分%.3f秒\n', degrees2dms(time_sum/3600));
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);
fprintf('平均时速: %.3fkm/h\n', distance_sum/time_sum*3.6);
% fprintf("组合导航轨迹与GNSS轨迹水平位置误差(RMS):%.3fm\n", positionerr_RMS);
