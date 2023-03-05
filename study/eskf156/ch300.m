close all;
clear;
clc;

addpath ./

format long g;
format compact;

N = 15;                 %ESKF维度
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;     % 重力加速度


ESKF156_FB_A = bitshift(1,0);
ESKF156_FB_V = bitshift(1,1);
ESKF156_FB_P = bitshift(1,2);
ESKF156_FB_W = bitshift(1,3); %反馈陀螺零篇
ESKF156_FB_G = bitshift(1,4); %反馈加计零篇
%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)

%% 相关选项及参数设置
opt.alignment_time = 10e2;      % 初始对准时间
opt.save_kml_enable = 0;                   %生成KML文件
opt.webmap_enable = 0;                     %生成卫星地图
opt.gravity_update_enable = 1;  % 使能重力静止量更新
opt.nhc_enable = 0;             % 车辆运动学约束
opt.zupt_enable = 0;              % ZUPT
opt.gnss_outage = 0;            % 模拟GNSS丢失
opt.outage_start = 900;         % 丢失开始时间(s)·
opt.outage_stop = 1280;          % 丢失结束时间(s)

opt.gnss_delay = 0;          % GNSS量测延迟 sec
opt.gnss_lever_arm = [-0.52; -1.30; 0.93]; %GNSS杆臂长度 b系下（右-前-上）
opt.gravity_R = 0.8;          % 重力更新 噪声
opt.nhc_R = 2.0;               % 车载非完整性约束噪声
opt.zuptR = 0.01;
opt.gnss_intervel = 1;          % GNSS间隔时间，如原始数据为10Hz，那么 gnss_intervel=10 则降频为1Hz

% 初始状态方差:    姿态           东北天速度  水平位置      陀螺零偏                              加速度计零偏
opt.P0 = diag([ [2 2 10]*D2R, [1 1 1], [5 5 5],  [50 50 50]* D2R/3600, [1e-2 1e-2 1e-3]*GRAVITY])^2;
% 系统方差:       角度随机游走           速度随机游走                      角速度随机游走        加速度随机游走
opt.Q = diag([(5/60*D2R)*ones(1,3), (4/60)*ones(1,3), 0*ones(1,3), [2.0 2.0 2.0]/3600*D2R, 30*1e-6*GRAVITY*ones(1,3)])^2;


%% 数据载入
load('dataset/2023年03月03日13时57分56秒_RAW.mat');
% load('dataset/2023年03月03日14时18分58秒_RAW.mat');
% load('dataset/2023年03月03日14时33分42秒_RAW.mat');

pos_type = data(:, 46);
evt_bit = data(:, 47);

imu_data = data(:, 21:26);
gnss_data = data(:, 27:44);
mcu_data = data(:, 3:20);
imu_length = length(imu_data);
gnss_length = length(gnss_data);
mcu_length = length(mcu_data);

imu_time = (data(:, 2) - data(1, 2));
mcu_time = imu_time;

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

mcu.lla = mcu_data(:, [8 7 9]);
mcu.vel = mcu_data(:, 4:6);
mcu.att = mcu_data(:, 1:3);

zupt_cntr = 0;
imu_dt = 0.01;
gnss_dt = imu_dt;
gyro_bias0 = mean(gyro_data(1:opt.alignment_time,:));

fprintf("gyro起始时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyro_bias0(1)*R2D, gyro_bias0(2)*R2D, gyro_bias0(3)*R2D);
gyro_bias_end = mean(gyro_data(end-opt.alignment_time:end, :));
fprintf("gyro结束时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyro_bias_end(1)*R2D, gyro_bias_end(2)*R2D, gyro_bias_end(3)*R2D);

%% EVT Bit
evt_gnss_mask = bitshift(1,0);
evt_gpst_mask = bitshift(1,2);
evt_ins_mask = bitshift(1,4);
evt_dual_mask = bitshift(1,5);

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
evt_dual = bitand(evt_bit, evt_dual_mask);

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
h0 = lla_data(1, 3);

time_sum = 0;
distance_sum = 0;
gnss_enu = zeros(gnss_length, 3);
log.vel_norm = zeros(gnss_length, 1);

% 根据速度 获得初始航向角
for i=1:gnss_length
    if norm(vel_data(i,:)) >5
        opt.inital_yaw = atan2(vel_data(i,1),vel_data(i,2));
        if(opt.inital_yaw < 0)
            opt.inital_yaw =  opt.inital_yaw + 360*D2R;
        end
        fprintf("初始航向角:%.2f°\r\n",  opt.inital_yaw*R2D);
        break;
    end
end
if i == gnss_length
    opt.inital_yaw = 0;
    fprintf("无法通过速度矢量找到初始航向角，设置为:%.2f°\r\n",  opt.inital_yaw*R2D);
end

for i=1:gnss_length
    [gnss_enu(i,1), gnss_enu(i,2), gnss_enu(i,3)] =  ch_LLA2ENU(lla_data(i,1), lla_data(i,2), lla_data(i,3), lat0, lon0, h0);
    log.vel_norm(i) = norm(vel_data(i, :));
    distance_sum = distance_sum + norm(vel_data(i, :))*gnss_dt;
    time_sum = time_sum + gnss_dt;
end

% gnss_enu = gnss_enu + normrnd(0,10, size(gnss_enu));
% vel_data = vel_data + normrnd(0,1, size(vel_data));

%% MCU结果转换为当地东北天坐标系
mcu_enu = zeros(mcu_length, 3);
for i=1:mcu_length
    [mcu_enu(i,1), mcu_enu(i,2), mcu_enu(i,3)] =  ch_LLA2ENU(mcu.lla(i,1)*D2R, mcu.lla(i,2)*D2R, mcu.lla(i,3), lat0, lon0, h0);
end

%% 初始参数设置
% 粗对准
g_b = - mean(acc_data(1:opt.alignment_time, :))';
g_b = g_b/norm(g_b);
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw;
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
log.std_acc_sldwin = zeros(imu_length, 1);
log.std_gyr_sldwin = zeros(imu_length, 1);
log.lambda = zeros(imu_length, 2); lambda = 1; pos_lambda = 1; vel_lambda = 1;
% log.lambda = zeros(imu_length, 6); lambda_save = [1,1,1,1,1,1];
log.gnss_r_ad = zeros(imu_length, 6); GNSS_R = zeros(6,6);

tic;
count_sec = 1;
for i=1:imu_length
    FB_BIT = 0; %反馈标志
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
    P = F*P*F' + opt.Q*imu_dt;
    
    %% 建立IMU滑窗,并求基本统计量
    if(i > 100)
        std_acc_sldwin = sum(std(acc_data(i-20:i, :)));
        std_gyr_sldwin = sum(std(gyro_data(i-20:i, :)));
        log.std_acc_sldwin(i) = std_acc_sldwin;
        log.std_gyr_sldwin(i) = std_gyr_sldwin;
    end
    
    % ZUPT计数器
    if(log.std_acc_sldwin(i) < 0.3 && log.std_gyr_sldwin(i) < 0.01 && norm(gyro_data(i,:)) < 0.5*D2R)
        zupt_cntr = zupt_cntr+1;
    else
        zupt_cntr = 0;
    end
    
    %% GNSS量测更新
    if (evt_gnss(i) && ~isnan(gnss_enu(i,1)))
        if( (~opt.gnss_outage || imu_time(i) < opt.outage_start || imu_time(i) > opt.outage_stop))
            if(norm(vel_std_data(i,:) - vel_std_data(i-1,:)) < 0.02 && norm(gnss_enu(i,:) - gnss_enu(i-1,:)) < 10000 && norm(vel_std_data(i,:))<1 ) % 踢掉GNSS输出的可能的不可靠结果
                
                nu = 10;
                h_m = 3;
                
                vel_H = zeros(3,15);
                vel_H(1:3,4:6) = eye(3);
                vel_Z = vel - vel_data(i,:)';
                vel_R = diag(vel_std_data(i,:))^2;
                Z = vel_Z; H = vel_H; Rk = vel_R;
                Py0 = H * P * H';
                rk = Z - H * X;
                vel_lambda = (nu + h_m)/(nu + rk'/(Py0 + Rk)*rk);
%                 vel_lambda = 1;
                vel_R = vel_R/vel_lambda;
                
                pos_H = zeros(3,15);
                pos_H(1:3,7:9) = eye(3);
                pos_Z = pos - gnss_enu(i,:)';
                pos_R = diag(pos_std_data(i,:))^2;
                Z = pos_Z; H = pos_H; Rk = pos_R;
                Py0 = H * P * H';
                rk = Z - H * X;
                pos_lambda = (nu + h_m)/(nu + rk'/(Py0 + Rk)*rk);
%                 pos_lambda = 1;
                pos_R = pos_R/pos_lambda;
                
                GNSS_R = diag([diag(vel_R); diag(pos_R)]);
                
                H = zeros(6,15);
                H(1:3,4:6) = eye(3);
                H(4:6,7:9) = eye(3);
                
                Z = [vel - vel_data(i,:)'; pos - gnss_enu(i,:)'];
                
                % GNSS量测延迟补偿
                Z = Z - [a_n; vel]*opt.gnss_delay;

                % GNSS天线杆壁效应补偿
                Z = Z + [-bCn*v3_skew(w_b); -bCn]*opt.gnss_lever_arm;
                
                % 量测6维集中自适应
%                 nu = 18;
%                 h_m = size(H,1);
%                 Py0 = H * P * H';
%                 rk = Z - H * X;
%                 Rk = GNSS_R;
%                 lambda = (nu + h_m)/(nu + rk'/(Py0 + Rk)*rk);
%                 GNSS_R = GNSS_R/lambda;
                
                R = GNSS_R;
                
                % 卡尔曼量测更新
                K = P * H' / (H * P * H' + R);
                X = X + K * (Z - H * X);
                P = (eye(N) - K * H) * P;
                
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
            end
        end
    end
    
%     log.lambda(i) = lambda;
    log.lambda(i,:) = [vel_lambda pos_lambda];
    log.gnss_r_ad(i,:) = sqrt(diag(GNSS_R));

    %% 静止状态下重力量测更新姿态
    if opt.gravity_update_enable && abs(((norm(acc_data(i,:))/GRAVITY)  - 1)) < 0.04
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
        FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
        FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
        FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
    end
    
    if opt.nhc_enable 
        % && norm(gyro_data(i,:)) < 5*D2R
        
        % 算法: Z定义在N系, 王博的
        %            H = zeros(3,15);
        %             H(1:3,4:6) = eye(3);
        %             vb_nhc = [log.vb(i,1); log.vb(i,2); log.vb(i,3)];
        %             vb_nhc(1) = 0;
        %             vb_nhc(3) = 0;
        %             vb_nhc = bCn * vb_nhc;
        %              Z = vel - vb_nhc;
        %             Rb = diag(ones(1, size(H, 1))*opt.nhc_R)^2;
        %             R = bCn*Rb*bCn';
        
        %算法2: Z定义在b系， 书上标准的
        H = zeros(2,15);
        A = [1 0 0; 0 0 1];
        H(1:2,4:6) = A*nCb;
        Z = 0 + (A*nCb)*vel;
        R = diag(ones(1, size(H, 1))*opt.nhc_R)^2;
        
        % 卡尔曼量测更新
        K = P * H' / (H * P * H' + R);
        X = X + K * (Z - H * X);
        P = (eye(N) - K * H) * P;
        FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
        FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
        FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
      %  FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
    end
    
    if opt.zupt_enable
        if zupt_cntr > 100
            log.zupt_state(i,1) = 1;
            
            H = zeros(3,15);
            H(1:3,4:6) = eye(3);
            
            Z = vel - 0;
            R = diag(ones(1, size(H, 1))*opt.zuptR)^2;
            
            % 卡尔曼量测更新
            K = P * H' / (H * P * H' + R);
            X = X + K * (Z - H * X);
            P = (eye(N) - K * H) * P;
            FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
        else
            log.zupt_state(i,1) = 0;
        end
    end
    
    
    % 姿态修正
    X_temp = X;
    
    if bitand(FB_BIT, ESKF156_FB_A)
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
    
    if bitand(FB_BIT, ESKF156_FB_V)
        vel = vel - X(4:6);
        X(4:6) = 0;
    end
    
    if bitand(FB_BIT, ESKF156_FB_P)
        pos = pos - X(7:9);
        X(7:9) = 0;
    end
    
    % 零偏反馈
    if bitand(FB_BIT, ESKF156_FB_W)
        gyro_bias = gyro_bias + X(10:12);
        X(10:12) = 0;
    end 
     
    if bitand(FB_BIT, ESKF156_FB_G)
        acc_bias = acc_bias + X(13:15);
        X(13:15) = 0;
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
    [kf_lla(i,1), kf_lla(i,2), kf_lla(i,3)] = ch_ENU2LLA(log.pos(i,1), log.pos(i,2) ,log.pos(i,3), lat0, lon0, h0);
end

%% IMU原始值
figure('name', 'IMU数据');
subplot(2,1,1);
plot(imu_time, acc_data, 'linewidth', 1.5); hold on; grid on;
ylabel('m/^(2)'); legend('X', 'Y','Z');
subplot(2,1,2);
plot(imu_time, gyro_data*R2D, 'linewidth', 1.5); hold on; grid on;
ylabel('deg');
if  opt.zupt_enable
    plot(imu_time, log.zupt_state*10, 'linewidth', 1.0); hold on; grid on;
    legend('X', 'Y','Z','ZUPT_STATE');
end

%% MCU ESKF
if size(data,2) == 59
    mcu_acc_bias = data(:, 57:59) / GRAVITY * 1000;
    mcu_gyr_bias = data(:, 54:56)*R2D;
    figure;
    subplot(2,1,1);
    plot(mcu_acc_bias, 'linewidth', 1.0);
    title("MCU ACC  零篇");  legend("X", "Y", "Z");  ylabel("零篇(mG)"); grid on;
    
    subplot(2,1,2, 'linewidth', 1.0);
    plot(mcu_gyr_bias);
    title("MCU GYR  零篇"); legend("X", "Y", "Z"); ylabel("零篇(dps)"); grid on;
end 

%% 姿态与航向估计曲线
plot_att(imu_time,log.att, mcu_time,mcu.att, imu_time,log.sins_att, imu_time,[bl_pitch bl_yaw]);

figure('name', "MGNSS组合导航航向与双天线航向");
subplot(2,1,1);
plot(imu_time, mcu.att(:,3),  imu_time, bl_yaw, '.-'); grid on;
xlim([imu_time(1) imu_time(end)]);
ylim([-10 370]);
yticks(0:45:360);
legend("MCU YAW", "GNSS DUAL YAW");
subplot(2,1,2);
plot(imu_time, atand(tand(mcu.att(:,3) - bl_yaw)));  grid on;
xlim([imu_time(1) imu_time(end)]);
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 速度估计曲线
plot_vel(imu_time,vel_data, imu_time,log.vel, mcu_time,mcu.vel);

%% 位置估计曲线
plot_enu(imu_time, gnss_enu, imu_time,log.pos, mcu_time, mcu_enu);

%% 二维轨迹
% plot_enu_2d(gnss_enu, log.pos);
plot_enu_2d(gnss_enu, log.pos, mcu_enu);

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
plot(imu_time, log.acc_bias(:, 1:3) / GRAVITY * 1000, 'linewidth', 1.5); grid on;
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

%%
% close all;

figure;
plot(imu_time, log.lambda); grid on;
xlim([imu_time(1) imu_time(end)]);
xlabel('时间(s)');

figure;
subplot(3,1,1);
plot(imu_time, log.gnss_r_ad(:,1), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, vel_std_data(:,1), 'LineWidth',1.5);
ylim([0 0.1]);
subplot(3,1,2);
plot(imu_time, log.gnss_r_ad(:,2), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, vel_std_data(:,2), 'LineWidth',1.5);
ylim([0 0.1]);
subplot(3,1,3);
plot(imu_time, log.gnss_r_ad(:,3), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, vel_std_data(:,3), 'LineWidth',1.5);
ylim([0 0.1]);
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

figure;
subplot(3,1,1);
plot(imu_time, log.gnss_r_ad(:,4), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, pos_std_data(:,1), 'LineWidth',1.5);
ylim([0 5]);
subplot(3,1,2);
plot(imu_time, log.gnss_r_ad(:,5), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, pos_std_data(:,2), 'LineWidth',1.5);
ylim([0 5]);
subplot(3,1,3);
plot(imu_time, log.gnss_r_ad(:,6), 'LineWidth',1.5); grid on; hold on;
plot(imu_time, pos_std_data(:,3), 'LineWidth',1.5);
ylim([0 5]);
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 数据统计
fprintf('行驶时间: %d小时%d分%.3f秒\n', degrees2dms(time_sum/3600));
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);

if opt.webmap_enable
    plot_google_map(lla_data*R2D, kf_lla*R2D, mcu.lla);
    plot_google_map(lla_data*R2D, kf_lla*R2D);
end

if opt.save_kml_enable
    kmlname = fullfile('gnss.kml');
    fprintf("写入GNSS数据到: %s\r\n", kmlname);
    kmlwriteline(kmlname,lla_data(:,1)*R2D,lla_data(:,2)*R2D, 'Color', 'blue');
    kmlwriteline(kmlname,kf_lla(:,1)*R2D,kf_lla(:,2)*R2D, 'Color', 'cyan');
end
