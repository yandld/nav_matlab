close all;
clear;
clc;

format long g;
format compact;

N = 17;             % ESKF维度
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;      % 重力加速度

%%
% bCn: b系到n系的旋转矩阵 Vn = bCn * Vb

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

%% 数据载入
% load('dataset/感图单天线2.2.9-7.4-路测数据1/感图单天线2.2.9-7.4-路测数据1.mat');
 % load('dataset/感图单天线9984-2.2.9-路测数据1/感图单天线9984-2.2.9-路测数据1.mat');
 %load('dataset/单天线-2.2.9-jul-4-地库1/单天线-2.2.9-jul-4-地库1.mat');
 % load('dataset/双天线rtk-2.2.9-jul-4-地库1/双天线rtk-2.2.9-jul-4-地库1.mat');
% load('dataset/240706_B/240706_B.mat');
% load('dataset/240710_A1/240710_A1.mat');
 % load('dataset/240710_A2/240710_A2.mat');
 % load('dataset/240710_B1/240710_B1.mat');
 %  load('dataset/240711A1/240711A1.mat');
  % load('dataset/240711A3/240711A3.mat');
 %  load('dataset/240712A1/240712A1.mat');
 %  load('dataset/240712D2/240712D2.mat');
%  load('dataset/240718A1/240718A1.mat');
% load('dataset/240718A2/240718A2.mat');
 % load('dataset/240718A3/240718A3.mat');
% load('dataset/240718B1/240718B1.mat');
% load('dataset/240718B2/240718B2.mat');
%load('dataset/240726B2里程计/240726B2里程计.mat');
% load('dataset/240727B1CAN/240727B1CAN.mat');
% load('dataset/240727B2/240727B2.mat');
% load('dataset/240805A/240805A1.mat');
%   load('dataset/240816A/240816A1.mat');
  load('dataset/240821C1/240821C1.mat');

%单位国际化
data.imu.acc =  data.imu.acc*GRAVITY;
data.imu.gyr =  data.imu.gyr*D2R;
data.gnss.lat = data.gnss.lat*D2R;
data.gnss.lon = data.gnss.lon*D2R;
data.dev.lat = data.dev.ins_lat*D2R;
data.dev.lon = data.dev.ins_lon*D2R;

%加入仿真噪声
% data.imu.acc(:,2) = data.imu.acc(:,2) + 0.1*GRAVITY;
%  data.imu.gyr(:,3) = data.imu.gyr(:,3) + 0.5*D2R;

att = [2 0 10]*D2R; %初始安装角
Cbrv = att2Cnb(att);%% from v to b 
bCv = Cbrv;% Cvb :b as subscript;v as superscript
%定义变量
ESKF156_FB_A = bitshift(1,0); %反馈失准角
ESKF156_FB_V = bitshift(1,1); %反馈速度
ESKF156_FB_P = bitshift(1,2); %反馈位置
ESKF156_FB_W = bitshift(1,3); %反馈陀螺零篇
ESKF156_FB_G = bitshift(1,4); %反馈加计零篇
ESKF156_FB_CBV = bitshift(1,5); %反馈安装角误差
gnss_vel_R = 0;
gnss_pos_R = 0;
gnss_lost_elapsed = 0;
gnss_last_valid_time = 0;
chi_lambda_vel = 999;
chi_lambda_pos = 999;
%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)

%% 相关选项及参数设置
opt.alignment_time = 1;          % 初始对准时间(s)
opt.gnss_outage = 0;             % 模拟GNSS丢失
opt.outage_start = 700;         % 丢失开始时间(s)
opt.outage_stop = 1000;          % 丢失结束时间(s)
opt.nhc_enable = 1;              % 车辆运动学约束
opt.nhc_lever_arm = 0*[0.35,0.35,-1.35]; %nhc杆臂长度 b系下（右-前-上）240816测试杆臂,仅测试天向，其他两轴为目测值
opt.nhc_R = 4.0;                % 车载非完整性约束噪声
opt.gnss_min_interval = 0;    % 设定时间间隔，例如0.5秒
opt.gnss_delay = 0;              % GNSS量测延迟 sec
opt.gnss_lever_arm = 0*[0.45;0.45;-1.34]; %GNSS杆臂长度 b系下（右-前-上）240816测试杆臂
opt.has_install_esti = 1;       %% can close or open ;1:esti insatllangle ; 0:no esti
q_bias_qrw = [25,50,50]*D2R/3600*2;
q_bias_vrw = [1 1 1.5]*1e-3*GRAVITY;
% 初始状态方差:    姿态       ENU速度  水平位置      陀螺零偏                加速度计零偏        安装俯仰角 安装航向角
% opt.P0 = diag([[2 2 10]*D2R, [1 1 1], [10 10 10], 0.1*D2R*ones(1,3), 1e-2*GRAVITY*ones(1,3), 10*D2R*ones(1,2) ])^2;
opt.P0 = diag([[5 5 10]*D2R, [1 1 1], [10 10 20], q_bias_qrw,q_bias_vrw , [10,10]*D2R])^2;
N = length(opt.P0);
% 系统误差:         角度随机游走          速度随机游走                     角速度随机游走            加速度随机游走
% opt.Q = diag([[0.1 0.1 0.1]*D2R/60, [0.1 0.1 0.2]/60, 0*ones(1,3), [0.001 0.0011 0.003]/3600*D2R, 50*1e-6/3600*GRAVITY*ones(1,3), [1 2]/3600*D2R])^2;
opt.Q = diag([[3 3 3]*D2R/60, [0.28 0.28 0.28]/60, 0*ones(1,9),[1 2]/3600*D2R])^2;
imu_len = length(data.imu.tow);
dev_len = length(data.dev.tow);

imu_dt = mean(diff(data.imu.tow));
gnss_dt = mean(diff(data.gnss.tow));

opt.nhc_upd_TS = 1/imu_dt;       % NHC upd period maybe used inthe future, nhc upd period 

% 开启静止时间
indices = find(data.imu.tow <= opt.alignment_time);  % 找到在x_seconds时间范围内的所有索引
acc_align0 = data.imu.acc(indices, :);
gyr_align0 = data.imu.gyr(indices, :);
gyr_bias0 = mean(gyr_align0);

% 结束静止时间
start_tow = data.imu.tow(end) - opt.alignment_time;  % 将x秒转换为毫秒，并从结束时间戳中减去
indices = find(data.imu.tow >= start_tow & data.imu.tow <= data.imu.tow(end));
gyr_bias_end = mean(data.imu.gyr(indices, :));

fprintf("gyro起始时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyr_bias0(1)*R2D, gyr_bias0(2)*R2D, gyr_bias0(3)*R2D);
fprintf("gyro结束时刻bias估计:%7.3f,%7.3f,%7.3f deg/s\n", gyr_bias_end(1)*R2D, gyr_bias_end(2)*R2D, gyr_bias_end(3)*R2D);
fprintf("IMU帧平均间隔:%.3fs\n", imu_dt);
fprintf("GNSS帧平均间隔:%.3fs\n", gnss_dt);

%% 经纬度转换为当地东北天坐标系
lat0 = data.gnss.lat(1);
lon0 = data.gnss.lon(1);
h0 = data.gnss.msl(1);

time_sum = 0;
distance_sum = 0;
gnss_enu = zeros(length(data.gnss.tow), 3);
log.vel_norm = zeros(length(data.gnss.tow), 1);

% 初始化上一次融合的时间
last_gnss_fusion_time = -inf;

inital_gnss_idx = 1;
% 根据速度 获得初始航向角
for i=1:length(data.gnss.tow)
    if norm(data.gnss.vel_enu(i,:)) > 1
        opt.inital_yaw = atan2(data.gnss.vel_enu(i,1),data.gnss.vel_enu(i,2));
        if(opt.inital_yaw < 0)
            opt.inital_yaw =  opt.inital_yaw + 360*D2R;
        end

        inital_gnss_idx = i;
        diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
        [~, inital_imu_idx] = min(diff_values);

        fprintf("初始航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, inital_imu_idx);
        break;
    end
end
if i == length(data.gnss.tow)
    opt.inital_yaw = 0;
    fprintf("无法通过速度矢量找到初始航向角，设置为:%.2f°\r\n",  opt.inital_yaw*R2D);
    inital_imu_idx = 1;
end


for i=inital_gnss_idx : length(data.gnss.tow)
    [gnss_enu(i,1), gnss_enu(i,2), gnss_enu(i,3)] =  ch_LLA2ENU(data.gnss.lat(i), data.gnss.lon(i), data.gnss.msl(i), lat0, lon0, h0);
    log.vel_norm(i) = norm(data.gnss.vel_enu(i, :));
    distance_sum = distance_sum + norm(data.gnss.vel_enu(i, :))*gnss_dt;
    time_sum = time_sum + gnss_dt;
end

%% MCU结果转换为当地东北天坐标系
dev_pos_enu = zeros(dev_len, 3);
for i=1:dev_len
    [dev_pos_enu(i,1), dev_pos_enu(i,2), dev_pos_enu(i,3)] =  ch_LLA2ENU(data.dev.lat(i), data.dev.lon(i),  data.dev.ins_msl(i), lat0, lon0, h0);
end

%% 初始参数设置
% 粗对准
g_b = - mean(acc_align0)';
g_b = g_b/norm(g_b);
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw;
pitch_sins = pitch0;
roll_sins = roll0;
yaw_sins = yaw0;
nQb = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
nQb_sins = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(N,1);
X_temp = X;
gyro_bias = X(10:12);
acc_bias = X(13:15);

P = opt.P0;

log.pitch = zeros(imu_len, 1);
log.roll = zeros(imu_len, 1);
log.yaw = zeros(imu_len, 1);
log.vel = zeros(imu_len, 3);
log.pos = zeros(imu_len, 3);
log.P = zeros(imu_len, N);
log.X = zeros(imu_len, N);
log.gyro_bias = zeros(imu_len, 3);
log.acc_bias = zeros(imu_len, 3);
log.sins_att = zeros(imu_len, 3);
log.vb = zeros(imu_len, 3);
log.installangle = zeros(imu_len, 3);
tic;
last_time = toc;
gnss_idx = inital_gnss_idx;
imucnt= 0;j=0;
imu_after_cal = nan(imu_len, 7);
% opt.nhc_enable = zeros(imu_len,1); %车辆运动学约束,后续需要记录用于对应发当前运动状态是否是判定准确
for i=inital_imu_idx:imu_len
    imucnt=imucnt+1;
    FB_BIT = 0; %反馈标志
    curr_time = toc;
    if curr_time - last_time >= 1 % 如果自上次更新已经过去了至少1秒
        fprintf('已完成 %.2f%%\n', (i / imu_len) * 100);
        last_time = curr_time; % 更新上次的时间
    end

    %% 捷联更新
    % 单子样等效旋转矢量法
    w_b = data.imu.gyr(i,:)';
    w_b = Cbrv*w_b;
    w_b = w_b - gyro_bias;

    f_b = data.imu.acc(i,:)';
    f_b = Cbrv*f_b;
    f_b = f_b - acc_bias;
    
    j = j+1;
    imu_after_cal(j,:)=[data.imu.tow(i),w_b',f_b'];

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
    F = zeros(N);
    F(4,2) = -f_n(3); %f_u天向比力
    F(4,3) = f_n(2); %f_n北向比力
    F(5,1) = f_n(3); %f_u天向比力
    F(5,3) = -f_n(1); %f_e东向比力
    F(6,1) = -f_n(2); %f_n北向比力
    F(6,2) = f_n(1); %f_e东向比力
    F(7:9, 4:6) = eye(3);
    F(1:3, 10:12) = -bCn;
    F(4:6, 13:15) =  bCn;
    F(16:17, 16:17) = -1/3600;%% 相关时间
    % 状态转移矩阵F离散化
    F = eye(N) + F*imu_dt;

    % 卡尔曼时间更新
    X = F*X;
    P = F*P*F' + opt.Q*imu_dt;

    current_time = data.imu.tow(i);
    gnss_lost_elapsed = current_time - gnss_last_valid_time;

    %% GNSS量测更新
    if gnss_idx <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(gnss_idx)) < 0.01 % threshold 是允许的最大差异

        if data.gnss.solq_pos(gnss_idx) > 0 && data.gnss.gnss_pos_std_n(gnss_idx) < 10%%40
            %gnss_vel_R = diag([0.1 0.1 0.1])^2;
            % gnss_pos_R = diag([1.2 1.2 1.2])^2;

            gnss_vel_R = diag(ones(3,1) * data.gnss.gnss_vel_std_n(gnss_idx))^2;
            gnss_pos_R = diag(ones(3,1) * data.gnss.gnss_pos_std_n(gnss_idx))^2;

            Hvel = zeros(3,N);
            Hvel(1:3, 4:6) = eye(3);
            Zvel = vel - data.gnss.vel_enu(gnss_idx,:)';
            Zvel = Zvel - a_n*opt.gnss_delay; % GNSS量测延迟补偿
            Zvel = Zvel + (-bCn*v3_skew(w_b))*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿

            Hpos = zeros(3,N);
            Hpos(1:3, 7:9) = eye(3);
            Zpos = pos - gnss_enu(gnss_idx,:)';

            Zpos = Zpos - vel*opt.gnss_delay; % GNSS量测延迟补偿
            Zpos = Zpos + (-bCn)*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿

            if (opt.gnss_outage == 0 || (opt.gnss_outage == 1 && (current_time < opt.outage_start || current_time > opt.outage_stop))) ...
                    && (current_time - last_gnss_fusion_time >= opt.gnss_min_interval)

                % Reference:  一种基于软卡方检测的自适应Ｋａｌｍａｎ滤波方法
                A = Hvel * P * Hvel' + gnss_vel_R;
                K = P * Hvel' / A;
                innov = Zvel - Hvel * X;

                chi_lambda_vel = innov'*A^(-1)*innov;
                log.lambda_vel(gnss_idx, :) = chi_lambda_vel;
                if(chi_lambda_vel < 1.5 || chi_lambda_pos < 0.15 || data.gnss.gnss_vel_std_n(gnss_idx) < 0.5 || data.gnss.gnss_pos_std_n(gnss_idx) < 5 || gnss_lost_elapsed > 3)
%                 if (chi_lambda_vel < 1.5) && (data.gnss.gnss_vel_std_n(gnss_idx) < 1) && (data.gnss.hdop(gnss_idx) < 0.5) && (data.gnss.nv(gnss_idx) > 25)
                    X = X + K * innov;
                    P = (eye(N) - K * Hvel) * P;
                    
                    gnss_last_valid_time = data.gnss.tow(gnss_idx);
                end

                % 位置更新
                A = Hpos * P * Hpos' + gnss_pos_R;
                K = P * Hpos' / (A);
                innov = Zpos - Hpos * X;

                chi_lambda_pos = innov'*A^(-1)*innov;
                log.lambda_pos(gnss_idx, :) = chi_lambda_pos;

%                 if(chi_lambda_pos < 0.15 && data.gnss.gnss_pos_std_n(gnss_idx) < 4) && (data.gnss.hdop(gnss_idx) < 0.5 && data.gnss.nv(gnss_idx) > 25)
                if(chi_lambda_vel < 1.5 || chi_lambda_pos < 0.15 || data.gnss.gnss_vel_std_n(gnss_idx) < 0.5 || data.gnss.gnss_pos_std_n(gnss_idx) < 5 || gnss_lost_elapsed > 3)
                    X = X + K * innov;
                    P = (eye(N) - K * Hpos) * P;
                    gnss_last_valid_time = data.gnss.tow(gnss_idx);
                end

                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);

                % 更新上一次融合的时间
                last_gnss_fusion_time = current_time;
            end
        end
        gnss_idx = gnss_idx + 1;
    end

    %% NHC 约束
%     if norm(vel) > 0.1 && gnss_lost_elapsed > 1 && norm(w_b) < 20*D2R
if(opt.nhc_enable)
    if norm(vel) > 1.5 && norm(w_b) < 10*D2R
%         if opt.nhc_enable
%             H = zeros(2,N);
%             A = [1 0 0; 0 0 1];
%             H(:,4:6) = A*nCb;
%             %  bCm = ch_eul2m([-X(16), 0, -X(17)]);
%             Z = 0 + (A*nCb)*vel;
%             R = diag(ones(1, size(H, 1))*opt.nhc_R)^2;
% 
%             % 卡尔曼量测更新
%             K = P * H' / (H * P * H' + R);
%             X = X + K * (Z - H * X);
%             P = (eye(N) - K * H) * P;
%             FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
%             FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
%             FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
%         end
              %  else % GNSS 有效
              %% v frame 速度误差作为安装角估计的量测方程 [Vvx_ins,Vvy_ins,Vvz_ins]=dVv = Vv'-Vv = -Cb2v*Cn2b*v3_skew(Vn)*dphi + v3_skew(Cb2v*Cn2b*Vn)*dinstallangle + Cb2v*Cn2b*dVn
              %% paper ref:车载MIMUGNSS组合高精度无缝导航关键技术研究--sunzhenqian
                    if (~opt.has_install_esti)
                       bCv = eye(3);
                    end
                    M1 = - bCv * nCb * v3_skew(vel);% v frame vel
                    % M = nCb * v3_skew(data.gnss.vel_enu(gnss_idx,:));
                    M2 = bCv * nCb;%M2 = Cb2v*Cn2b
                    M3 = v3_skew(bCv * log.vb(i,:)');% M3=v3_skew(Cb2v*Cn2b*Vn)
                    H = zeros(2, N);
        
                    H(1, 1:3) = M1(1,:);
                    H(1, 4:6) = M2(1,:);
                    H(1, 17)  = M3(1,3);
                    H(2, 1:3) = M1(3,:);
                    H(2, 4:6) = M2(3,:);
                    H(2, 16)  = M3(3,1);
        
                    Vn2v_sins = bCv * log.vb(i, 1:3)';

                    Z = Vn2v_sins([1,3]);
        
                    R = blkdiag(0.1, 0.1)^2;
        
                    % 卡尔曼量测更新
                    K = P * H' / (H * P * H' + R);
                    X = X + K * (Z - H * X);
                    P = (eye(N) - K * H) * P;
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                    if (opt.has_install_esti)
                       FB_BIT = bitor(FB_BIT, ESKF156_FB_CBV);
                    end
                    
               %  end
    end
end
    % 状态暂存
    X_temp = X;

    if bitand(FB_BIT, ESKF156_FB_A)
        rv = X(1:3);
        rv_norm = norm(rv);
        if rv_norm > 0
            qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
            nQb = ch_qmul(qe, nQb);
            nQb = ch_qnormlz(nQb); %单位化四元数
            bQn = ch_qconj(nQb); %更新bQn
            bCn = ch_q2m(nQb); %更新bCn阵
            nCb = bCn'; %更新nCb阵
            X(1:3) = 0;
        end

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

    if bitand(FB_BIT, ESKF156_FB_CBV)
        cvv = att2Cnb([X(16),0,X(17)]);
        bCv = cvv*bCv;
        X(16:17) = 0;
        att = m2att(bCv);        
    end
    


    %% 信息存储
    [pitch, roll, yaw] = q2att(nQb);
    log.pitch(i,:) = pitch;
    log.roll(i,:) = roll;
    log.yaw(i,:) = yaw;
    log.installangle(i,:)=att'*180/pi;
    log.vel(i,:) = vel';
    log.pos(i,:) = pos';
    log.X(i, :) = X_temp';
    log.P(i, :) = sqrt(diag(P))';
    log.gyro_bias(i, :) = gyro_bias;
    log.acc_bias(i, :) = acc_bias;
    log.tow(i,:) = current_time;

    % 纯惯性信息存储
    [pitch_sins, roll_sins, yaw_sins] = q2att(nQb_sins);
    log.sins_att(i,:) = [pitch_sins roll_sins yaw_sins];
end
imu_after_cal(j+1:end,:) = [];

fprintf('数据处理完毕，用时%.3f秒\n', toc);

set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
set(groot, 'defaultAxesZGrid', 'on');
%% 姿态与航向估计曲线
% plot_att(data.imu.tow,log.att, mcu_time,mcu.att, data.imu.tow,log.sins_att, data.imu.tow,[bl_pitch bl_yaw]);
%
% figure('name', "MGNSS组合导航航向与双天线航向");
% subplot(2,1,1);
% plot(data.imu.tow, mcu.att(:,3),  data.imu.tow, bl_yaw, '.-'); grid on;
% xlim([data.imu.tow(1) data.imu.tow(end)]);
% ylim([-10 370]);
% yticks(0:45:360);
% legend("MCU YAW", "GNSS DUAL YAW");
% subplot(2,1,2);
% plot(data.imu.tow, atand(tand(mcu.att(:,3) - bl_yaw)));  grid on;
% xlim([data.imu.tow(1) data.imu.tow(end)]);
% set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 位置
figure('name', '位置');
subplot(2,3,1);
plot(data.imu.tow, log.pos(:,1), '.-'); hold on;
plot(data.imu.tow, data.dev.pos_enu(:,1), '.-');hold on;
title('Pos East'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,4);
plot(data.imu.tow, log.P(:,7), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_pos(:,1), '.-');
title('Pos East Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,2);
plot(data.imu.tow, log.pos(:,2), '.-'); hold on;
plot(data.imu.tow, data.dev.pos_enu(:,2), '.-');
title('Pos North'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,5);
plot(data.imu.tow, log.P(:,8), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_pos(:,2), '.-');
title('Pos North Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,3);
plot(data.imu.tow, log.pos(:,3), '.-'); hold on;
plot(data.imu.tow, data.dev.pos_enu(:,3), '.-');
title('Pos Up'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,6);
plot(data.imu.tow, log.P(:,9), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_pos(:,3), '.-');
title('Pos Up Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 速度
figure('name', '速度');
subplot(2,3,1);
plot(data.imu.tow, log.vel(:,1), '.-'); hold on;
plot(data.imu.tow, data.dev.vel_enu(:,1), '.-');
title('Vel East'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,4);
plot(data.imu.tow, log.P(:,4), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_vel(:,1), '.-');
title('Vel East Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,2);
plot(data.imu.tow, log.vel(:,2), '.-'); hold on;
plot(data.imu.tow, data.dev.vel_enu(:,2), '.-');
title('Vel North'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,5);
plot(data.imu.tow, log.P(:,5), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_vel(:,2), '.-');
title('Vel North Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,3);
plot(data.imu.tow, log.vel(:,3), '.-'); hold on;
plot(data.imu.tow, data.dev.vel_enu(:,3), '.-');
title('Vel Up'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,6);
plot(data.imu.tow, log.P(:,6), '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_vel(:,3), '.-');
title('Vel Up Std'); xlabel('时间(s)'); ylabel('m'); legend('MATLAB','DEV'); xlim tight;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%%  姿态
figure('name', '姿态');
subplot(2,3,1);
plot(data.imu.tow, log.roll, '.-'); hold on;
plot(data.imu.tow, data.dev.roll, '.-');
title('Roll'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,4);
plot(data.imu.tow, log.P(:,1)*R2D, '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_att(:,1), '.-');
title('Roll Std'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,2);
plot(data.imu.tow, log.pitch, '.-'); hold on;
plot(data.imu.tow, data.dev.pitch, '.-');
title('Pitch'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,5);
plot(data.imu.tow, log.P(:,2)*R2D, '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_att(:,2), '.-');
title('Pitch Std'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;

subplot(2,3,3);
plot(data.imu.tow, log.yaw(:,1), '.-'); hold on;
plot(data.imu.tow, data.dev.yaw, '.-');
title('Yaw'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;
subplot(2,3,6);
plot(data.imu.tow, log.P(:,3)*R2D, '.-'); hold on;
plot(data.imu.tow, data.dev.kf_p_att(:,3), '.-');
title('Yaw Std'); xlabel('时间(s)'); ylabel('deg'); legend('MATLAB','DEV'); xlim tight;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% IMU零偏估计曲线
figure('name', 'IMU零偏估计曲线');
subplot(2,2,1);
color_rgb = get(gca,'ColorOrder');
plot(data.imu.tow, log.gyro_bias(:, 1)  * R2D, 'Color', color_rgb(1,:), 'linewidth', 1.5); hold on;
plot(data.imu.tow, log.gyro_bias(:, 2)  * R2D, 'Color', color_rgb(2,:), 'linewidth', 1.5);
plot(data.imu.tow, log.gyro_bias(:, 3)  * R2D, 'Color', color_rgb(3,:), 'linewidth', 1.5);
plot(data.imu.tow, data.dev.kf_wb(:, 1)  * R2D, '-', 'Color', color_rgb(1,:), 'linewidth', 0.5);
plot(data.imu.tow, data.dev.kf_wb(:, 2)  * R2D, '-', 'Color', color_rgb(2,:), 'linewidth', 0.5);
plot(data.imu.tow, data.dev.kf_wb(:, 3)  * R2D, '-', 'Color', color_rgb(3,:), 'linewidth', 0.5);
plot(data.imu.tow, gyr_bias0(1)  * R2D * ones(imu_len,1), '-.', 'Color', color_rgb(1,:), 'linewidth', 0.3);
plot(data.imu.tow, gyr_bias0(2)  * R2D * ones(imu_len,1), '-.', 'Color', color_rgb(2,:), 'linewidth', 0.3);
plot(data.imu.tow, gyr_bias0(3)  * R2D * ones(imu_len,1), '-.', 'Color', color_rgb(3,:), 'linewidth', 0.3);
xlim tight;
title('GYR零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(dps)'); legend('MATLAB_X', 'MATLAB_Y', 'MATLAB_Z', 'Dev_X', 'Dev_Y', 'Dev_Z');

subplot(2,2,2);
plot(data.imu.tow, log.acc_bias(:, 1) / GRAVITY * 1000, 'Color', color_rgb(1,:), 'linewidth', 1.5); hold on;
plot(data.imu.tow, log.acc_bias(:, 2) / GRAVITY * 1000, 'Color', color_rgb(2,:), 'linewidth', 1.5);
plot(data.imu.tow, log.acc_bias(:, 3) / GRAVITY * 1000, 'Color', color_rgb(3,:), 'linewidth', 1.5);
plot(data.imu.tow, data.dev.kf_gb(:, 1) / GRAVITY * 1000, '-', 'Color', color_rgb(1,:), 'linewidth', 0.5);
plot(data.imu.tow, data.dev.kf_gb(:, 2) / GRAVITY * 1000, '-', 'Color', color_rgb(2,:), 'linewidth', 0.5);
plot(data.imu.tow, data.dev.kf_gb(:, 3) / GRAVITY * 1000, '-', 'Color', color_rgb(3,:), 'linewidth', 0.5);
xlim tight;
title('ACC零偏估计曲线'); xlabel('时间(s)'); ylabel('零偏(mg)'); legend('MATLAB_X', 'MATLAB_Y', 'MATLAB_Z', 'Dev_X', 'Dev_Y', 'Dev_Z');

subplot(2,2,3);
semilogy(data.imu.tow, log.P(:, 10:12)  * R2D, 'linewidth', 1.5); grid on;
xlim tight;
title('GYR零偏协方差收敛曲线(MATLAB)'); xlabel('时间(s)'); ylabel('零偏标准差(dps)'); legend('X', 'Y', 'Z');

subplot(2,2,4);
semilogy(data.imu.tow, log.P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
xlim tight;
title('加速度计零偏协方差收敛曲线'); xlabel('时间(s)'); ylabel('零偏标准差(mg)'); legend('X', 'Y', 'Z');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 状态量曲线
figure('name','状态量曲线');
subplot(2,2,1);
plot(data.imu.tow, log.X(:, 1) * R2D, 'c', 'linewidth', 1.5); hold on; grid on;
plot(data.imu.tow, log.X(:, 2) * R2D, 'm', 'linewidth', 1.5);
plot(data.imu.tow, log.P(:, 1) * R2D * 1, 'r-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 2) * R2D * 1, 'g-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 1) * R2D * -1, 'r-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 2) * R2D * -1, 'g-.', 'linewidth', 1);
xlim tight;
ylim([-0.5 0.5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Orientation','horizontal');

subplot(2,2,3);
plot(data.imu.tow, log.X(:, 3) * R2D, 'c', 'linewidth', 1.5); hold on; grid on;
plot(data.imu.tow, log.P(:, 3) * R2D * 1, 'b-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 3) * R2D * -1, 'b-.', 'linewidth', 1);
xlim tight;
ylim([-5 5]);
xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Yaw', 'Orientation','horizontal');

subplot(2,2,2);
plot(data.imu.tow, log.X(:, 4:6), 'linewidth', 1.5); hold on; grid on;
plot(data.imu.tow, log.P(:, 4:6)  * 1, '-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 4:6)  * -1, '-.', 'linewidth', 1);
xlim tight;
ylim([-10 10]);
xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('E', 'N', 'U', 'Orientation','horizontal');

subplot(2,2,4);
plot(data.imu.tow, log.X(:, 7:9), 'linewidth', 1.5); hold on; grid on;
plot(data.imu.tow, log.P(:, 7:9) * 1, '-.', 'linewidth', 1);
plot(data.imu.tow, log.P(:, 7:9) * -1, '-.', 'linewidth', 1);
xlim tight;
ylim([-100 100]);
xlabel('时间(s)'); ylabel('位置误差(m)'); legend('E', 'N', 'U', 'Orientation','horizontal');

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

figure('name', 'GNSS接收机给出的信息');
subplot(2,2,1);
plot(data.gnss.tow, data.gnss.gnss_vel_std_n); hold on; grid on;
plot(data.gnss.tow, log.lambda_vel);
xlabel('时间(s)'); legend('gnss_vel_std_n', 'log.lambda_vel', 'Orientation','horizontal');

subplot(2,2,2);
plot(data.gnss.tow, data.gnss.gnss_pos_std_n); hold on; grid on;
plot(data.gnss.tow, log.lambda_pos);
xlabel('时间(s)'); legend('gnss_pos_std_n', 'log.lambda_pos', 'Orientation','horizontal');

subplot(2,2,3);
plot(data.gnss.tow, data.gnss.hdop); grid on;
xlabel('时间(s)'); legend('hdop', 'Orientation','horizontal');

subplot(2,2,4);
plot(data.gnss.tow, data.gnss.nv); grid on;
xlabel('时间(s)'); legend('卫星数', 'Orientation','horizontal');

%% 安装误差角在线估计结果
figure('name', '安装误差角在线估计结果');
subplot(2,2,1);
plot(data.imu.tow, log.installangle(:,1), 'LineWidth', 1.5); grid on;
xlabel('时间(s)'); ylabel('俯仰安装角(°)'); xlim tight;
subplot(2,2,3);
plot(data.imu.tow, log.P(:, 16)*R2D, 'LineWidth', 1.5); grid on;
xlabel('时间(s)'); ylabel('俯仰安装角方差(°)'); xlim tight;

subplot(2,2,2);
plot(data.imu.tow, log.installangle(:,3), 'LineWidth', 1.5); grid on;
xlabel('时间(s)'); ylabel('航向安装角(°)'); xlim tight;
subplot(2,2,4);
plot(data.imu.tow, log.P(:, 17)*R2D, 'LineWidth', 1.5); grid on;
xlabel('时间(s)'); ylabel('航向安装角方差(°)'); xlim tight;
% 
% subplot(111);
% plot(data.imu.tow, log.installangle(:,2), 'LineWidth', 1.5); grid on;
% xlabel('时间(s)'); ylabel('滚动安装角(°)'); xlim tight;

set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 二维轨迹
figure('name', '2D轨迹');
plot_enu(gnss_enu);
plot_enu(log.pos);
plot_enu(dev_pos_enu);

legend("GNSS", "MATLAB", "DEV嵌入式设备轨迹");
axis equal;
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%% 数据统计
time_duration = seconds(time_sum);
time_duration.Format = 'hh:mm:ss';

fprintf('行驶时间: %s\n', char(time_duration));
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);


%% 转换为kml
[filepath, name, ~] = fileparts(scriptPath);
[lat, lon, h] = ch_ENU2LLA(log.pos(:,1),log.pos(:,2),log.pos(:,3),lat0, lon0, h0);
rgbColor = [255, 0, 0];
kmlFileName = fullfile(filepath, name + "_MATLAB.kml");
generateKmlFiles(kmlFileName, lat*R2D, lon*R2D, 20, rgbColor);

