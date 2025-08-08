close all;
clear;
clc;

format long g;
format compact;
N = 20;             % ESKF维度
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;      % 重力加速度

%% 相关选项及参数设置
opt.alignment_time = 1;          % 初始对准时间(s)
opt.gnss_outage = 0;             % 模拟GNSS丢失
opt.outage_start = 300;         % 丢失开始时间(s)
opt.outage_stop = 400;          % 丢失结束时间(s)
opt.nhc_enable = 1;              % 车辆运动学约束
opt.nhc_lever_arm = 0*[0.40;0.00;0.55]; %nhc杆臂长度 b系下（右-前-上）240816测试杆臂,仅测试天向，其他两轴为目测值
opt.nhc_R = 1;                % 车载非完整性约束噪声
opt.gnss_min_interval = 0;    % 设定时间间隔，例如0.5秒
opt.gnss_delay = 0.000;              % GNSS量测延迟 sec
opt.gnss_lever_arm = 1*[-0.04;0.0;0.4]; %GNSS杆臂长度 b系下（右-前-上）240816测试杆臂
opt.has_install_esti = 1;       %% can close or open ;1:esti insatllangle ; 0:no esti
opt.cali_status = 0;%设置标定标志位 0：未标定，1：标定中，2标定完成
opt.zupt_enable = 1;              % 启用ZUPT
opt.zupt_vel_threshold = 2.5;     % ZUPT速度阈值(m/s)
opt.zupt_gyr_threshold = 0.5*D2R; % ZUPT角速度阈值(rad/s)
opt.zupt_time_threshold = 3.0;    % ZUPT持续时间阈值(s)
opt.zupt_R = diag(5*[0.1, 0.1, 0.1].^2); % ZUPT测量噪声协方差
opt.init_yaw_time = 5;
opt.zaru_R = diag(D2R*[0.1, 0.1, 0.01].^2); % ZARU测量噪声协方差
opt.od_use_flag = 0;% 0：不使用轮速，1：使用轮速
opt.use_dualgnss_flag = 1;
opt.anta = 0;%双天线安装角
opt.pos_feeddbacks_factor = 0.005;
opt.pos_partial_feedback_enble = 0;
%%
% Cb2n: b系到n系的旋转矩阵 Vn = Cb2n * Vb

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

%% 数据载入
% load("dataset\2024-12-23-单天线1\2024-12-23-单天线1.mat");
load("dataset\HI43_247_808_dual-11-26-24\HI43_247_808_dual-11-26-24.mat");
%load("dataset\HI43-4-15-APP245-2-15-07-45\HI43-4-15-APP245-2-15-07-45.mat");
data.gnss.gnss_delay = data.gnss.gnss_delay;
data.imu.tow = data.imu.tow - 0.000;
%单位国际化
data.imu.acc =  data.imu.acc*GRAVITY;
data.imu.gyr =  data.imu.gyr*D2R;
data.gnss.lat = data.gnss.lat*D2R;
data.gnss.lon = data.gnss.lon*D2R;
data.dev.lat = data.dev.ins_lat*D2R;
data.dev.lon = data.dev.ins_lon*D2R;
data.dev.cali_status = opt.cali_status;%从配置参数读取当前标定状态，方便进行下一步决策

att = [0 0 0] * D2R; %初始安装角
Cb2v_simulate = att2Cnb(att); % matlab仿真设置的安装角误差
att = [0 0 0] * D2R;
Cb2v = eye(3);
Cv2b = Cb2v';
%定义变量
ESKF156_FB_A = bitshift(1,0); %反馈失准角
ESKF156_FB_V = bitshift(1,1); %反馈速度
ESKF156_FB_P = bitshift(1,2); %反馈位置
ESKF156_FB_W = bitshift(1,3); %反馈陀螺零篇
ESKF156_FB_G = bitshift(1,4); %反馈加计零篇
ESKF156_FB_CBV = bitshift(1,5); %反馈安装角误差
ESKF156_FB_LEVER = bitshift(1,6); %反馈杆臂误差
gnss_vel_R = 0;
gnss_pos_R = 0;
gnss_lost_elapsed = 0;
gnss_last_valid_time = 0;
chi_lambda_vel = 999;
chi_lambda_pos = 999;
zupt_detect_time = 0;
is_zupt = 0;
install_yaw_dual2V = 0;
pos_std_pre = 0;
loggnss.solq = 0;
gn = [0 0 1];
dual_heading = 0;
reverse_heading_flag = 0;%0:正向,1:倒车

%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3) IMU安装角(2) 杆臂(2) 时间(1)

opt.Q = zeros(N,N);
opt.P0 = zeros(N,N);
opt.P1 =  zeros(N,N);
% 初始状态方差:    姿态       ENU速度  水平位置      陀螺零偏       加速度计零偏        安装俯仰角 安装航向角
opt.P0 = diag([[2 2 10]*D2R, [1 1 1], [5 5 5], 0.11*D2R*[1,1,1], 0.001*GRAVITY*ones(1,3), 1*D2R*[2 20],0.0*[1,1,0.1]])^2;
opt.P1 = diag([[2 2 10]*D2R, [1 1 1], [5 5 5], 0.11*D2R*[1,1,1], 0.001*GRAVITY*[1,1,1], 0*D2R*[2 20] ,0.0*[1,1,1]])^2;
% N = length(opt.P0);
% 系统误差:         角度随机游走          速度随机游走                     角速度随机游走            加速度随机游走
opt.Q = diag([(0.05*D2R)*[1,1,1], (0.01)*ones(1,3), 0*ones(1,3), 2.0/3600*D2R*[1,1,0.1], 25/1e6*GRAVITY*ones(1,3), 0.0001/3600*[1 10]*D2R , 0.00001/3600*[1,1,1]])^2;
imu_len = length(data.imu.tow);
dev_len = length(data.dev.tow);
gnss_len = length(data.gnss.tow);
imu_dt = mean(diff(data.imu.tow));
gnss_dt = mean(diff(data.gnss.tow));

% 开启静止时间
indices = find(data.imu.tow <= opt.alignment_time);  % 找到在x_seconds时间范围内的所有索引
acc_align0 = data.imu.acc(indices, :);
gyr_align0 = data.imu.gyr(indices, :);
gyr_bias0 = mean(gyr_align0);

% 结束静止时间
start_tow = data.imu.tow(end) - opt.alignment_time;  % 将x秒转换为毫秒，并从结束时间戳中减去
indices = find(data.imu.tow >= start_tow & data.imu.tow <= data.imu.tow(end));
gyr_bias_end = mean(data.imu.gyr(indices, :));

fprintf("gyro起始时刻bias估计:%7.3f,%7.3f,%7.3f deg/h\n", gyr_bias0(1)*R2D*3600, gyr_bias0(2)*R2D*3600, gyr_bias0(3)*R2D*3600);
fprintf("gyro结束时刻bias估计:%7.3f,%7.3f,%7.3f deg/h\n", gyr_bias_end(1)*R2D*3600, gyr_bias_end(2)*R2D*3600, gyr_bias_end(3)*R2D*3600);
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

yaw_flag = zeros(imu_len,1);%初始化航向标识位0：未获取到初始航向；1：已经获取到初始航向
init_yaw_count = 0;
inital_gnss_idx = 0;
% 根据速度 获得初始航向角
%imu初始化航向前imu数据姿态和速度解算
g_b = - mean(acc_align0)';
g_b = Cb2v_simulate * g_b/norm(g_b);
pitch00 = zeros(imu_len,1);
roll00 = zeros(imu_len,1);
yaw00 = zeros(imu_len,1);
pitch00(1) = asin(-g_b(2));
roll00(1) = atan2( g_b(1), -g_b(3));
Qb2n = angle2quat(-yaw00(1), pitch00(1), roll00(1), 'ZXY');
vel0 = zeros(imu_len,3)';
pos0 = zeros(imu_len,3)';
zupt_detect_time0 = 0;
is_zupt0 = 0;
a = 12;
%% test，定义一阶kf滤波P,Q,R;
install_yaw_dual2V_filter = 0;P_DUAL2V = 100 * D2R;Q_DUAL2V = 0 * D2R;R_DUAL2V = 10 * D2R;%GNSS速度和dual航向角估计安装角的PQR
od_scale_factor_P = 0.5;od_scale_factor_Q = 0;od_scale_factor_R = 0.005;
vel_norm0 = 0;
err = [0 0 0]';
%%
for i=1:length(data.imu.tow)
    
    w_b = data.imu.gyr(i,:)';
    w_b = w_b - gyr_bias0';
    w_b = Cb2v_simulate*w_b;
    
    f_b = data.imu.acc(i,:)';
    f_b = Cb2v_simulate*f_b;
    dv(1:2) = [0  0];
    if abs(norm(f_b) - GRAVITY)<0.05 && norm(data.gnss.vel_enu(inital_gnss_idx+1,:)) <0.1
        w_b = w_b - 0.01*err;
    end
    if i>1
        if is_zupt0==0
            [Qb2n, pos0(:,i), vel0(:,i), ~] = inertial_navigation_update(w_b, f_b, Qb2n, pos0(:,i-1), vel0(:,i-1), GRAVITY, imu_dt);
        else
            [Qb2n, pos0(:,i), vel0(:,i), ~] = inertial_navigation_update(w_b, f_b, Qb2n, pos0(:,i-1), vel0(:,i-1), GRAVITY, 0);
        end
        [pitch00(i), roll00(i), yaw00(i)] = q2att(Qb2n);
        
        %mahony 启动前维持roll,pitch
        norm_f_b = f_b/norm(f_b);
        Qn2b = Qb2n .* [1 -1 -1 -1];
        gb = ch_qmulv(Qn2b, gn)';
        err = cross(norm_f_b , gb);
        a_n = ch_qmulv(Qb2n, f_b);
        log.err(i,:) =err';
    end
   
    %判断载体航向初始化前静止
    
    vel_norm0 = norm(vel0(:,i));
    gyr_norm = norm(w_b);
    if vel_norm0 < opt.zupt_vel_threshold && gyr_norm < opt.zupt_gyr_threshold *2
        zupt_detect_time0 = zupt_detect_time0 + imu_dt;
        if zupt_detect_time0 >= opt.zupt_time_threshold
            is_zupt0 = true;
        end
    else
        zupt_detect_time0 = 0;
        is_zupt0 = false;
    end
    if is_zupt0 || (norm(data.gnss.vel_enu(inital_gnss_idx+1,:)) <0.1 && data.gnss.solq_pos(inital_gnss_idx+1)>0)
        vel0(:,i) = [0 0 0]';
    end

    %gnss数据获取初始速度
    if (inital_gnss_idx+1) <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(inital_gnss_idx+1)) < 0.01 % threshold 是允许的最大差异 
        inital_gnss_idx = inital_gnss_idx + 1;
        if norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1 
            init_yaw_count = init_yaw_count +1;
            opt.inital_yaw = atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2));
            %opt.inital_yaw = opt.inital_yaw - (atan2(vel0(1,i),vel0(2,i)) - yaw00(i)*D2R) ;
            opt.inital_yaw = yaw_convert_0_2pi(opt.inital_yaw); %角度范围转换
            
            %根据速度和双天线航向估计双天线安装角度---start
            if data.gnss.solq_heading(inital_gnss_idx,1)  == 4
                 dual_heading = atan2(data.gnss.dual_enu(inital_gnss_idx,1),data.gnss.dual_enu(inital_gnss_idx,2));
                 vel_heading = atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2));
                 install_yaw_dual2V = (vel_heading - dual_heading + opt.anta *D2R);%估计双天线初始安装方向
                 
                 [install_yaw_dual2V,~] = yaw_convert_npi_pi(install_yaw_dual2V);%角度范围转换
                 if install_yaw_dual2V_filter == 0 && install_yaw_dual2V < 30 * D2R %
                     install_yaw_dual2V_filter = install_yaw_dual2V;
                 end
                 if (norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1) && install_yaw_dual2V < 30 * D2R
                    [install_yaw_dual2V_filter, P_DUAL2V, ~, ~] = kf_measurement_update(install_yaw_dual2V_filter, P_DUAL2V, install_yaw_dual2V, 1, R_DUAL2V);
                 end 
            end
            %根据速度和双天线航向估计双天线安装角度---end
            log.install_yaw_dual2V_filter(inital_gnss_idx,1) = install_yaw_dual2V_filter;
            %单天线初始化航向
            if(init_yaw_count >opt.init_yaw_time/0.1 ) || (norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 3 && 0.5*init_yaw_count >opt.init_yaw_time/0.1) %GNSS-10hz计算
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                
                fprintf("单天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                fprintf("速度估计IMU安装角:%.2f°, v系初始航向:%f\r\n",  R2D * yaw_convert_0_2pi(atan2(vel0(1,i),vel0(2,i))-yaw00(i)*D2R), atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2))*57);
                fprintf("启动IMU速度：%f,%f",vel0(1,i),vel0(2,i));
                break;
            %双天线初始化航向
            elseif data.gnss.solq_heading(inital_gnss_idx,1)  == 4 && opt.use_dualgnss_flag
                
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                opt.inital_yaw = atan2(data.gnss.dual_enu(inital_gnss_idx,1),data.gnss.dual_enu(inital_gnss_idx,2));
                [yaw_diff , ~ ] = yaw_convert_npi_pi(opt.inital_yaw - (vel_heading + opt.anta*D2R));
                if norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1 &&  abs(yaw_diff)<30 * D2R
                    opt.inital_yaw = yaw_convert_0_2pi(opt.inital_yaw - opt.anta*D2R - (atan2(vel0(1,i),vel0(2,i)) - yaw00(i)*D2R));
                    fprintf("双天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                    break;
                end
           
            end
        elseif data.gnss.solq_heading(inital_gnss_idx,1)  == 4 && opt.use_dualgnss_flag && i > 1
                
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                [opt.inital_yaw, ~] = yaw_convert_0_2pi(atan2(data.gnss.dual_enu(inital_gnss_idx,1),data.gnss.dual_enu(inital_gnss_idx,2)));
                fprintf("双天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                break; 
            %init_yaw_count=0;
        end
    
    end
end
initial_id = i;
%航向初始化失败
if i == length(data.gnss.tow)
    opt.inital_yaw = 0;
    fprintf("无法通过速度矢量找到初始航向角，设置为:%.2f°\r\n",  opt.inital_yaw*R2D);
    inital_imu_idx = 1;
    
end
%位置速度转换
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
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw;
Qb2n = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(N,1);
gyr_bias = gyr_bias0';
acc_bias = X(13:15);
%AVP初始化完成，根据标定状态修改标定位并选择进入标定状态或者正常导航状态
if data.dev.cali_status == 0
    data.dev.cali_status = 1;%如果未设置标定完成，则进入标定状态
    P = opt.P0;
elseif data.dev.cali_status == 2
    P = opt.P1;%标定主要是标定安装角和杆臂，目前只标安装角，防止初始安装角过大，将P1的安装角误差适当增大    
end

% 双天线安装角误差估计一阶KF参数
P_dual_angle = (10 * D2R).^2;
Q_dual_angle = (0 * D2R).^2;
R_dual_angle = (1.5 * D2R).^2;
%% 记录数据
log.pitch = zeros(imu_len, 1);
log.roll = zeros(imu_len, 1);
log.yaw = zeros(imu_len, 1);
log.vel = zeros(imu_len, 3);
log.Z_vel = zeros(imu_len, 3);
log.pos = zeros(imu_len, 3);
log.Z_pos = zeros(imu_len, 3);
log.P = zeros(imu_len, N);
log.X = zeros(imu_len, N);
log.gyr_bias = zeros(imu_len, 3);
log.acc_bias = zeros(imu_len, 3);
log.sins_att = zeros(imu_len, 3);
log.vb = zeros(imu_len, 3);
log.installangle = zeros(imu_len, 3);
log.Z_dual_heading = zeros(imu_len, 1);
log.gnss_lever_arm = zeros(imu_len, 3);
log.gnss_delay = zeros(imu_len, 1);
log.Z_od_speed = zeros(imu_len, 1);
log.od_scale_factor = zeros(imu_len, 1);
Z_dual_heading = 0;
tic;
last_time = toc;
gnss_idx = inital_gnss_idx;
imucnt= 0;
Zvel = 0;
Zpos = 0;
log.is_gnss_update = zeros(imu_len, 1);
Z = [0;0];
Vn2v_sins = [0;0;0];
od_scale_factor = 1;
for i=inital_imu_idx:imu_len
    imucnt=imucnt+1;
    FB_BIT = 0; %反馈标志
    curr_time = toc;
    if curr_time - last_time >= 1 % 如果自上次更新已经过去了至少1秒
        fprintf('已完成 %.2f%%\n', (i / imu_len) * 100);
        last_time = curr_time; % 更新上次的时间
    end

    %% 捷联更新(注入仿真安装角和 反馈的 acc/gyr bias)
    w_b = data.imu.gyr(i,:)';
    w_b = Cb2v_simulate*w_b;
    w_b = w_b - gyr_bias;

    f_b = data.imu.acc(i,:)';
    f_b = Cb2v_simulate*f_b;
    f_b = f_b - acc_bias;

    [Qb2n, pos, vel, ~] = inertial_navigation_update(w_b, f_b, Qb2n, pos, vel, GRAVITY, imu_dt);
    [~, ~, yaw] = q2att(Qb2n);

    %% 卡尔曼滤波
    Cb2n = ch_q2m(Qb2n); %更新Cb2n阵
    Cn2b = Cb2n'; %更新Cn2b阵

    [X, P, f_n] = imu_propagation(X, P, f_b, Cb2n, opt.Q, imu_dt);

    a_n = f_n + [0; 0; -GRAVITY];
    log.vb(i, :) = (Cn2b * vel)';

    current_time = data.imu.tow(i);
    gnss_lost_elapsed = current_time - gnss_last_valid_time;

    %% GNSS量测更新
    if gnss_idx <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(gnss_idx)) < 0.01  % threshold 是允许的最大差异
        % 判断是否进行GNSS更新
        %根据速度和双天线航向估计双天线安装角度---start
        if data.gnss.solq_heading(gnss_idx,1)  == 4  && norm(data.gnss.vel_enu(gnss_idx))>1
             dual_heading = atan2(data.gnss.dual_enu(gnss_idx,1),data.gnss.dual_enu(gnss_idx,2));
             vel_heading = atan2(data.gnss.vel_enu(gnss_idx,1),data.gnss.vel_enu( gnss_idx,2));
             while dual_heading < 0
                dual_heading = dual_heading +360*D2R;
             end
             install_yaw_dual2V = (vel_heading - dual_heading + opt.anta*D2R);%估计双天线初始安装方向
             [install_yaw_dual2V,~] = yaw_convert_npi_pi(install_yaw_dual2V);
             if (norm(data.gnss.vel_enu(gnss_idx,:)) > 1) && install_yaw_dual2V < 30 * D2R
                [install_yaw_dual2V_filter, P_DUAL2V, ~, ~] = kf_measurement_update(install_yaw_dual2V_filter, P_DUAL2V, install_yaw_dual2V, 1, R_DUAL2V);
             end 
             
        end
%         根据速度和双天线航向估计双天线安装角度---end
        update_gnss = false;
        if data.gnss.solq_pos(gnss_idx) > 0

            gnss_vel_n = norm(data.gnss.vel_enu(gnss_idx, 1:3));
            loggnss.solq = data.gnss.solq_pos(gnss_idx);
            Hvel = zeros(3,N);
            Hvel(1:3, 4:6) = eye(3);
%             Hvel(1:3,1:3) = - Cb2n*v3_skew(v3_skew(w_b)*opt.gnss_lever_arm);
%             Hvel(1:3, 10:12) = -Cb2n*skew(opt.gnss_lever_arm);
            Zvel = vel - data.gnss.vel_enu(gnss_idx,:)';
            Zvel = Zvel - a_n * (opt.gnss_delay + data.gnss.gnss_delay(gnss_idx)/1000 ); % GNSS量测延迟补偿
            Zvel = Zvel + (Cb2n*v3_skew(w_b))*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿

            Hpos = zeros(3,N);
            Hpos(1:3, 7:9) = eye(3);
%             Hpos(1:3, 1:3) = Cb2n*skew(opt.gnss_lever_arm);
            Zpos = pos - gnss_enu(gnss_idx,:)';
            
            Zpos = Zpos - vel*(opt.gnss_delay + data.gnss.gnss_delay(gnss_idx)/1000 ) + 0.0 * 0.04*0.04 * a_n; % GNSS量测延迟补偿
            Zpos = Zpos + (Cb2n)*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿
            if N==20 %估计杆臂xy和时间差
                Hvel(1:3, 18:20) = -Cb2n*v3_skew(w_b);
                Hpos(1:3, 18:20) = -(Cb2n);
                Hvel(1:3, 20) = a_n;
                Hpos(1:3, 20) = vel;
            end
            gnss_vel_R = diag(ones(3,1) * data.gnss.gnss_vel_std_n(gnss_idx) + norm(a_n) * 0.01)^2 * 1;
            gnss_pos_R = diag(ones(3,1) * (data.gnss.gnss_pos_std_n(gnss_idx) + norm(vel) * 0.001))^2 * 1;
            if (is_zupt)
                gnss_pos_R = gnss_pos_R *25;
            end
            X_pre = X(a);%研究各个观测量对kf的状态影响
            if (opt.gnss_outage == 0 || (opt.gnss_outage == 1 && (current_time < opt.outage_start || current_time > opt.outage_stop))) ...
                    && (current_time - last_gnss_fusion_time >= opt.gnss_min_interval) 

                if isfield(data.gnss, 'dual_enu')   
                    dual_enu_n = data.gnss.dual_enu(gnss_idx, :);
                   
                    if data.gnss.solq_heading(gnss_idx,1) == 4  %判断条件加入航向固定解解算,还应考虑时间不同步带来的航向误差
                        %叉乘计算速度向量和基线向量夹角
                        if norm(data.gnss.vel_enu(gnss_idx, 1:2))>1 && data.dev.cali_status < 3 && 0
                            Z_angle_gv = asin((data.gnss.vel_enu(gnss_idx, 1)*dual_enu_n(2) - data.gnss.vel_enu(gnss_idx, 2) *dual_enu_n(1) )/(norm(data.gnss.vel_enu(gnss_idx,1:2))*norm(dual_enu_n(1:2))));
                            [install_yaw_dual2V, P_dual_angle, ~, ~] = kf_measurement_update(install_yaw_dual2V, P_dual_angle+Q_dual_angle, Z_angle_gv, 1, R_dual_angle);
                        end
                        %计算航向和误差
                        [~,dual_heading] = yaw_convert_0_2pi(atan2(dual_enu_n(1),dual_enu_n(2)));%双天线航向
                        
                        Z_dual_heading = yaw  + (att(3)*R2D) + opt.anta - (dual_heading + (install_yaw_dual2V_filter) *R2D + w_b(3)*(opt.gnss_delay+data.gnss.gnss_delay(gnss_idx)/1000 - 0.015) * R2D );%考虑yaw角的坐标系b或v
                        [~,Z_dual_heading] = yaw_convert_npi_pi(Z_dual_heading*D2R);%角度范围转换
                        
                        R_dual_heading = (0.5 * D2R).^2;%设置航向噪声DEG
                        H_dual_heading = zeros(1,N);
                        H_dual_heading(1,3) = 1;
                        H_dual_heading(1,17) = 1;
%%%%%%%
                        if data.dev.cali_status > 0 && norm(w_b)<3*D2R && norm(data.gnss.vel_enu(gnss_idx, 1:2))<1.1  && sqrt(abs(P(17,17))) < 0.5 * D2R && 1 %标定完成状态才可以使用双天线航向信息，否则双天线航向会混入imu安装角中
                            [X, P, ~, ~] = kf_measurement_update(X, P, Z_dual_heading*D2R, H_dual_heading, R_dual_heading);
                            X_pre = X(a);
                            FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                            X(16:17) = 0;
                        end
                    end
                end

                % 卡方：  Reference:  一种基于软卡方检测的自适应Ｋａｌｍａｎ滤波方法
                [chi_lambda_vel, ~] = calculate_chi_square(X, P, Hvel, Zvel, gnss_vel_R);
                log.lambda_vel(gnss_idx, :) = sqrt(chi_lambda_vel);%转为m/s

                [chi_lambda_pos, ~] = calculate_chi_square(X, P, Hpos, Zpos, gnss_pos_R);
                log.lambda_pos(gnss_idx, :) = sqrt(chi_lambda_pos);%转为m
                log.gnss_vel_chi(gnss_idx,:) = data.gnss.gnss_vel_std_n(gnss_idx)^2 * chi_lambda_vel;
                log.gnss_pos_chi(gnss_idx,:) = data.gnss.gnss_pos_std_n(gnss_idx)^2 * chi_lambda_pos;
                % 计算速度和位置的不确定性
                vel_uncertainty_std = sqrt(norm(P(4:6, 4:6)));
                pos_uncertainty_std = sqrt(norm(P(7:9, 7:9)));
                pos_std_diff = data.gnss.gnss_vel_std_n(gnss_idx) - pos_std_pre;
                pos_std_pre = data.gnss.gnss_vel_std_n(gnss_idx);

                if (chi_lambda_vel < 0.5 && chi_lambda_pos < 0.15 ) || (data.gnss.gnss_vel_std_n(gnss_idx) < 0.3 && data.gnss.gnss_pos_std_n(gnss_idx) < 5)      
                    update_gnss = true;
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                elseif (vel_uncertainty_std /data.gnss.gnss_vel_std_n(gnss_idx) > 0.5 || pos_uncertainty_std / data.gnss.gnss_pos_std_n(gnss_idx) > 0.05)
                    update_gnss = true;
                    
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                end
                
                % 执行GNSS更新
                if update_gnss && (current_time < opt.outage_start || current_time > opt.outage_stop)
                    if norm(data.gnss.vel_enu(gnss_idx,:))>0.3 || sqrt(norm(P(7:9, 7:9))) < 1 * D2R%载体震动上电982速度不准确
                        [X, P, ~, ~] = kf_measurement_update(X, P, Zvel, Hvel, gnss_vel_R);
                        X_pre = X(a);
                    end
                    
                    [X, P, ~, ~] = kf_measurement_update(X, P, Zpos, Hpos, gnss_pos_R);
                    X_pre = X(a);
                    gnss_last_valid_time = data.gnss.tow(gnss_idx);
%                    
                end
             end

                % 杆臂零偏和时间延迟全部设置为安装角收敛后反馈
                acc_bias_converged = all(diag(P(13:14, 13:14)) < (0.008 * GRAVITY)^2);
%                 gyr_bias_converged = all(diag(P(17, 17)) < (0.6*D2R)^2) ;
                gyr_bias_converged = 1;
                lever_arm_converged = gyr_bias_converged;
                % 确保 FB_BIT 是无符号整数类型
                FB_BIT = uint32(FB_BIT);

                % 只有当加速度计零偏收敛时，才反馈加速度计零偏
                if acc_bias_converged  || 1%&& norm(w_b) < 2*D2R
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_G));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_G))));
                    X(13:15) = 0;
                end

                if gyr_bias_converged %&& norm(w_b) < 2*D2R
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_W));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_W))));
                    X(10:12) = 0;
                end

                if lever_arm_converged ||1%&& norm(w_b) < 2*D2R
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_LEVER));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_LEVER))));
                    X(18:20) = 0;
                end
                
                % 更新上一次融合的时间
                last_gnss_fusion_time = current_time;
        end
        gnss_idx = gnss_idx + 1;
    
    end

    %% NHC 约束
    if(opt.nhc_enable && 1)
        norm_vel = norm(vel);
        
        if (norm_vel > 0.1  && (gnss_lost_elapsed >= 1 && norm(w_b) < 200*D2R)) || (gnss_lost_elapsed < 1 && norm(w_b) < 20*D2R && norm_vel > 0.5) %%
            %v frame 速度误差作为安装角估计的量测方程 [Vvx_ins,Vvy_ins,Vvz_ins]=dVv = Vv'-Vv = -Cb2v*Cn2b*v3_skew(Vn)*dphi + v3_skew(Cb2v*Cn2b*Vn)*dinstallangle + Cb2v*Cn2b*dVn
            % paper ref:车载MIMUGNSS组合高精度无缝导航关键技术研究--sunzhenqian
            if (~opt.has_install_esti)vb
                Cb2v = eye(3);
            end
            
            Vn2v_sins = Cb2v * log.vb(i, 1:3)' + (Cb2n*v3_skew(w_b))*opt.nhc_lever_arm;
            %理论上正向前进y轴速度应为正，否则安装角估计为其补角，修正后可以利用判断低速倒车
            if Vn2v_sins(2) < -1 && norm(Vn2v_sins)>3
                angle = atan2(Vn2v_sins(1),Vn2v_sins(2));
                Cb2v = Cb2v * att2Cnb([0 0 pi]);
                Vn2v_sins = Cb2v * log.vb(i, 1:3)';
            end
            %根据Y轴速度判断倒车
            if Vn2v_sins(2)< -0.2 
                reverse_heading_flag = 1;
            elseif Vn2v_sins(2)> -0.01
                reverse_heading_flag = 0;
            end
            M1 = - Cb2v * Cn2b * v3_skew(vel);% v frame vel
            % M = Cn2b * v3_skew(data.gnss.vel_enu(gnss_idx,:));
            M2 = Cb2v * Cn2b;%M2 = Cb2v*Cn2b
            M3 = v3_skew(Cb2v * log.vb(i,:)');% M3=v3_skew(Cb2v*Cn2b*Vn)
            H = zeros(2, N);
            
            H(1, 1:3) = M1(1,:);
            H(1, 4:6) = M2(1,:);
            H(1, 17)  = M3(1,3);
            H(2, 1:3) = M1(3,:);
            H(2, 4:6) = M2(3,:);
            H(2, 16)  = M3(3,1);
            w_v = Cb2v * w_b;
            Z = Vn2v_sins([1,3]);
            R = diag(ones(1, size(H, 1))*opt.nhc_R)^2;

            [X, P, ~, ~] = kf_measurement_update(X, P, Z, H, R);
            X_pre = X(a);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
            if (opt.has_install_esti)  && norm(w_b) < 3*D2R && update_gnss %设置安装角反馈阈值
                FB_BIT = bitor(FB_BIT, ESKF156_FB_CBV);
            elseif sqrt(P(17:17))<0.5*D2R
                %由于全程使用NHC反馈，其大角度速率NHC会导致安装角估计不准确，此时不反馈安装角并需要清除安装角反馈
                X(16) = 0;
                X(17) = 0;
            end
        end
    end
    log.X_INSTALL(i,:) = X(16:17)'*R2D;
    %% OD轮速检测和更新
    if(opt.od_use_flag && 0)
        
        M1 = - Cb2v * Cn2b * v3_skew(vel);% v frame vel
        % M = Cn2b * v3_skew(data.gnss.vel_enu(gnss_idx,:));
        M2 = Cb2v * Cn2b;%M2 = Cb2v*Cn2b
        M3 = v3_skew(Cb2v * log.vb(i,:)');% M3=v3_skew(Cb2v*Cn2b*Vn)
        Vn2v_sins = Cb2v * log.vb(i, 1:3)' + (Cb2n*v3_skew(w_b))*opt.nhc_lever_arm;
        %轮速比例因子估计
        if Vn2v_sins(2)>1 && gnss_lost_elapsed < 1 && data.od.speed(i) >= 5
            od_scale_factor_tempt = Vn2v_sins(2) / (data.od.speed(i) + 0.5);
            [od_scale_factor, od_scale_factor_P, ~, ~] = kf_measurement_update(od_scale_factor, od_scale_factor_P, od_scale_factor_tempt, 1, od_scale_factor_R);
        end
        H = zeros(1, N);
%         H(1,1:3) = M1(2,1:3);
        H(1,4:6) = M2(2,1:3);
%         H(1,16) = M3(2,1);
%         H(1,17) = M3(2,3);
        Z_od_speed = Vn2v_sins(2) - od_scale_factor * (data.od.speed(i));%0.5是车速舍去小数部分的补偿
        R = 0.6;
        if gnss_lost_elapsed > 1 && Vn2v_sins(2)>1 && 1
            [X, P, ~, ~] = kf_measurement_update(X, P, Z_od_speed, H, R);
                
            FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
        end
    end
    %% ZUPT检测和更新
    if opt.zupt_enable
        vel_norm = norm(vel);
        gyr_norm = norm(w_b);

        if vel_norm < opt.zupt_vel_threshold && gyr_norm < opt.zupt_gyr_threshold && vel_norm < 20*norm(diag(P(4:6,4:6)))+0.02
            zupt_detect_time = zupt_detect_time + imu_dt;
            if zupt_detect_time >= opt.zupt_time_threshold
                is_zupt = true;
            end
        else
            zupt_detect_time = 0;
            is_zupt = false;
        end

        if is_zupt 
            % ZUPT更新
            is_zupt = false;
            if gnss_lost_elapsed < 1 &&  gnss_vel_n<0.3
%                 zupt_detect_time = 0;
                H = zeros(3, N);
                H(:, 4:6) = eye(3);  % 速度误差状态
                z_zupt = vel;  % 观测值：当前速度（应该接近零）
    %              fprintf('ZUPT生效，当前速度为%fm/s;速度误差P为%fm/s.\r\n',norm(vel),norm(diag(P(4:6,4:6))));
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zupt, H, opt.zupt_R);
             %ZARU
                H = zeros(3, N);
                H(:, 10:12) = eye(3);  % 速度误差状态
                z_zaru = w_b;  % 观测值：当前速度（应该接近零
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zaru, H, opt.zaru_R);
    
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
            elseif gnss_lost_elapsed > 1
                %             zupt_detect_time = 0;
                H = zeros(3, N);
                H(:, 4:6) = eye(3);  % 速度误差状态
                z_zupt = vel;  % 观测值：当前速度（应该接近零）
    %              fprintf('ZUPT生效，当前速度为%fm/s;速度误差P为%fm/s.\r\n',norm(vel),norm(diag(P(4:6,4:6))));
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zupt, H, opt.zupt_R);
                %ZARU
                H = zeros(3, N);
                H(:, 10:12) = eye(3);  % 速度误差状态
                z_zaru = w_b;  % 观测值：当前速度（应该接近零
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zaru, H, opt.zaru_R);
    
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);

            end
        end
    end
    %% 
    if data.dev.cali_status <2 ...
            && sqrt(abs(P(17,17))) < 0.6 * D2R
        data.dev.cali_status = 2;
        fprintf('标定完成：%f',i);
    end
    log.X(i, :) = X';
    %% 反馈
    if bitand(FB_BIT, ESKF156_FB_A)
        rv = X(1:3);
        rv_norm = norm(rv);
        if rv_norm > 0 && reverse_heading_flag==0
            qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
            Qb2n = ch_qmul(qe, Qb2n);
            Qb2n = ch_qnormlz(Qb2n); %单位化四元数
            Cb2n = ch_q2m(Qb2n); %更新Cb2n阵
            Cn2b = Cb2n'; %更新Cn2b阵
            X(1:3) = 0;
        end

    end

    if bitand(FB_BIT, ESKF156_FB_V)
        if update_gnss == 0 && 0
            vel = vel - 1*X(4:6);
            X(4:6) = 0;
        else 
            vel = vel - X(4:6);
            X(4:6) = 0;
        end
    end

    if bitand(FB_BIT, ESKF156_FB_P)
        if gnss_lost_elapsed >5 && norm(X(7:9)) > opt.pos_feeddbacks_factor && opt.pos_partial_feedback_enble
            pos = pos - opt.pos_feeddbacks_factor/norm(X(7:9))*X(7:9);
            X(7:9) = X(7:9) -  opt.pos_feeddbacks_factor/norm(X(7:9))*X(7:9);
        else
        fd_factor = 0.01;
        pos = pos - fd_factor*X(7:9);
        X(7:9) = (1 - fd_factor)*X(7:9);
        end
    end

    % 零偏反馈
    if bitand(FB_BIT, ESKF156_FB_W) 
        gyr_bias = gyr_bias + X(10:12);
        X(10:12) = 0;
    end

    if bitand(FB_BIT, ESKF156_FB_G)
        acc_bias = acc_bias + X(13:15);
        X(13:15) = 0;
    end

    if bitand(FB_BIT, ESKF156_FB_CBV)
        cvv = att2Cnb([X(16),0,X(17)]);
        Cb2v = cvv*Cb2v;
        Cv2b = Cb2v';
        X(16:17) = 0;
        att = m2att(Cb2v);
    end
    
    if bitand(FB_BIT, ESKF156_FB_LEVER) 
       opt.gnss_lever_arm(1:2) = opt.gnss_lever_arm(1:2) + X(18:19);
       opt.gnss_delay = opt.gnss_delay + X(20);
       X(18:20) = 0;
    end
    %% 信息存储
    [pitch, roll, yaw] = q2att(Qb2n); %考虑输出航向IMU系还是车体系，输出车体系姿态应转换到V车体系
%     [attv,~]= m2att(Cb2n*Cv2b);
%     pitch = attv(1)*R2D; roll = attv(2)*R2D; 
%     yaw = yaw_convert_0_2pi(-attv(3))*R2D; %车体系欧拉角
    log.pitch(i,:) = pitch;
    log.roll(i,:) = roll;
    log.yaw(i,:) = yaw;
    log.installangle(i,:)=att'*180/pi;
    log.vel(i,:) = vel';
    log.Z_vel(i,:) = Zvel;
    log.pos(i,:) = pos';
    log.Z_pos(i,:) = Zpos;
    log.P(i, :) = sqrt(diag(P))';
    log.gyr_bias(i, :) = gyr_bias;
    log.acc_bias(i, :) = acc_bias;
    log.tow(i,:) = current_time;
    log.Z_dual_heading(i,:) = Z_dual_heading;
    log.install_yaw_dual2V(i,:) = install_yaw_dual2V*R2D;
    log.install_yaw_dual2V_filter(i,:) = install_yaw_dual2V_filter*R2D;
    log.Z_NHC(i,:) = Z';
    log.gnss_solq(i) = loggnss.solq';
    log.is_gnss_update(i,:) = update_gnss;
    log.gnss_lever_arm(i,:) = opt.gnss_lever_arm';
    log.gnss_delay(i) = opt.gnss_delay;
    log.dual_heading(i,:) = dual_heading;
    log.Vn2v_sins(i,:) = Vn2v_sins';
    log.reverse_heading_flag(i,:) = reverse_heading_flag;
    log.od_scale_factor(i,:) = od_scale_factor;
    %log.Z_od_speed(i,:) = Z_od_speed;
end
vel0=vel0';
pos0=pos0';
fprintf('数据处理完毕，用时%.3f秒\n', toc);

set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
set(groot, 'defaultAxesZGrid', 'on');

%% P
navigation_plots.axis_comparison('XLabel', '时间(s)', ...
    'YLabel', {{'East (m)', 'North (m)', 'Up (m)'}, {'Std East (m)', 'Std North (m)', 'Std Up (m)'}}, ...
    'Title', '位置对比', ...
    'Time', {data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow}, ...
    'Values', {log.pos, data.dev.pos_enu, log.P(:,7:9), data.dev.kf_p_pos}, ...
    'Labels', {'Matlab', 'HI30', 'Matlab_P', 'HI30_P'}, ...
    'Subplots', [1, 1, 2, 2]);

%% V
navigation_plots.axis_comparison('XLabel', '时间(s)', ...
    'YLabel', {{'East (m/s)', 'North (m/s)', 'Up (m/s)'}, {'Std East (m/s)', 'Std North (m/s)', 'Std Up (m/s)'}}, ...
    'Title', '速度对比', ...
    'Time', {data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow}, ...
    'Values', {log.vel, data.dev.vel_enu, log.P(:,4:6), data.dev.kf_p_vel}, ...
    'Labels', {'Matlab', 'HI30', 'Matlab_P', 'HI30_P'}, ...
    'Subplots', [1, 1, 2, 2]);
%% A
navigation_plots.axis_comparison('XLabel', '时间(s)', ...
    'YLabel', {{'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'}, {'Std Roll (deg)', 'Std Pitch (deg)', 'Std Yaw (deg)'}}, ...
    'Title', '姿态对比', ...
    'Time', {data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow}, ...
    'Values', {[log.roll log.pitch log.yaw], [data.dev.roll data.dev.pitch data.dev.yaw], log.P(:,1:3)*R2D, data.dev.kf_p_att*R2D}, ...
    'Labels', {'Matlab', 'HI30', 'Matlab_P', 'HI30_P'}, ...
    'Subplots', [1, 1, 2, 2]);
%% WB
navigation_plots.axis_comparison('XLabel', '时间(s)', ...
    'YLabel', {{'X (deg)', 'Y (deg)', 'Z (deg)'}, {'Std X (deg)', 'Std Y (deg)', 'Std Z (deg)'}}, ...
    'Title', '陀螺零偏对比', ...
    'Time', {data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow}, ...
    'Values', {log.gyr_bias*R2D, repmat(gyr_bias0*R2D, length(data.imu.tow), 1), data.dev.kf_wb*R2D, log.P(:, 10:12)*R2D, data.dev.kf_p_wb*R2D}, ...
    'Labels', {'Matlab', 'bias0', 'HI30', 'Matlab_P', 'HI30_P'}, ...
    'Subplots', [1, 1, 1, 2, 2], ...
    'LineStyles', {'-', '-.', ':', '-'});
%% GB
navigation_plots.axis_comparison('XLabel', '时间(s)', ...
    'YLabel', {{'X (mG)', 'Y (mG)', 'Z (mG)'}, {'Std X (mG)', 'Std Y (mG)', 'Std Z (mG)'}}, ...
    'Title', '加计零偏对比', ...
    'Time', {data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow, data.imu.tow}, ...
    'Values', {log.acc_bias*1000/GRAVITY, data.dev.kf_gb*1000/GRAVITY, log.P(:, 13:15)*1000/GRAVITY, data.dev.kf_p_gb*1000/GRAVITY}, ...
    'Labels', {'Matlab', 'HI30', 'Matlab_P', 'HI30_P'}, ...
    'Subplots', [1, 1, 2, 2]);


%% 状态量曲线
navigation_plots.multi_plot(...
    'Time', data.imu.tow, ...
    'Values', {log.X(:, 1) * R2D, log.X(:, 2) * R2D, log.X(:, 3) * R2D, log.X(:, 4), log.X(:, 5), log.X(:, 6), log.X(:, 7), log.X(:, 8), log.X(:, 9)},...
    'LegendLabels', { 'Pitch', 'Roll', 'Yaw','E', 'N', 'U','E', 'N', 'U'},...
    'Titles', {'失准角XY','失准角Z','速度','位置'}, ...
    'XLabel', '时间(s)', ...
    'YLabels', {'deg', 'deg', 'm/s', 'm'}, ...
    'FigureName', '状态量曲线', ...
    'Layout', [2, 3], ...
    'Subplots', [1, 1, 2, 3, 3, 3, 4, 4, 4]);

%% 纯GNSS信息
navigation_plots.multi_plot('Time', data.gnss.tow, ...
    'Values', {data.gnss.gnss_vel_std_n, log.lambda_vel, data.gnss.gnss_pos_std_n, log.lambda_pos, data.gnss.hdop, data.gnss.nv}, ...
    'LegendLabels', {'gnss_vel_std_n', 'lambda_vel', 'gnss_pos_std_n', 'lambda_pos', 'hdop', '卫星数'}, ...
    'Titles', {'gnss_vel_std_n与lambda_vel', 'gnss_pos_std_n与lambda_pos', 'hdop', '卫星数'}, ...
    'XLabel', '时间 (s)', ...
    'YLabels', {'m', 'm', ' ', ' '}, ...
    'FigureName', '纯GNSS与卡方检验', ...
    'Layout', [2, 2], ...
    'Subplots', [1, 1, 2, 2, 3, 4]);

%% 安装角
navigation_plots.multi_plot('Time', data.imu.tow, ...
    'Values', {log.installangle(:,1), log.installangle(:,3), log.P(:, 16)*R2D, log.P(:, 17)*R2D}, ...
    'LegendLabels', {'Pitch', 'Yaw', 'Pitch_Std', 'Yaw_Std'}, ...
    'Titles', {'Pitch', 'Yaw', 'Pitch_Std', 'Yaw_Std'}, ...
    'XLabel', '时间 (s)', ...
    'YLabels', {'deg', 'deg', 'deg', 'deg'}, ...
    'FigureName', '安装误差角在线估计结果', ...
    'Layout', [2, 2], ...
    'Subplots', [1, 2, 3, 4]);
%% LEVER and Time
navigation_plots.multi_plot('Time', data.imu.tow, ...
    'Values', {log.gnss_lever_arm(:,1), log.gnss_lever_arm(:,2), log.gnss_delay(:,1)}, ...
    'LegendLabels', {'杆臂X', '杆臂Y', 'IMU/GNSS时间差'}, ...
    'Titles', {'杆臂X，Y', '时间'}, ...
    'XLabel', '时间 (s)', ...
    'YLabels', {'m/米', 'm/米', 's/秒'}, ...
    'FigureName', '杆臂和时间同步在线估计结果', ...
    'Layout', [1, 2], ...
    'Subplots', [1, 1, 2]);
%% 2D
navigation_plots.trajectory_2d_plot({gnss_enu, log.pos, dev_pos_enu}, {'GNSS', 'MATLAB', 'DEV嵌入式设备轨迹'});

for i=1:imu_len
    [matlab.lat(i), matlab.lon(i), matlab.h(i)] = ch_ENU2LLA2(log.pos(i,1), log.pos(i,2), log.pos(i,3), lat0, lon0, h0);
end
for i=1:length(data.gnss.tow)
    [gnss.lat(i),gnss.lon(i), gnss.h(i)] = ch_ENU2LLA2(gnss_enu(i,1), gnss_enu(i,2), gnss_enu(i,3), lat0, lon0, h0);
end
for i=1:imu_len
    [dev.lat(i),dev.lon(i), dev.h(i)] = ch_ENU2LLA2(dev_pos_enu(i,1), dev_pos_enu(i,2), dev_pos_enu(i,3), lat0, lon0, h0);
end
% wm = webmap('World Imagery');
% wmline(matlab.lat*R2D, matlab.lon*R2D,'Color','r','Width',3);
% wmline(gnss.lat*R2D, gnss.lon*R2D,'Color','b','Width',3);
% wmline(dev.lat*R2D, dev.lon*R2D,'Color','g','Width',3);
% %% 3D
% figure;
% scatter3(matlab.lon*R2D, matlab.lat*R2D, data.imu.tow);  % 颜色基于时间大小
%% 数据统计
time_duration = seconds(time_sum);
time_duration.Format = 'hh:mm:ss';

fprintf('行驶时间: %s\n', char(time_duration));
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);

figure;
plot(diff(log.pos));
legend('东向位置差分','北向位置差分','天向位置差分');

figure;
plot(diff(data.dev.pos_enu))
legend('东向位置差分','北向位置差分','天向位置差分');

%% dynamicDisplay(log, data, 播放速度, 播放开始时间s);
% dynamicDisplay(log, data, 100, 10);
dynamicDisplayGUI(log, data);
%%
function [chi_lambda, innov] = calculate_chi_square(X, P, H, Z, R)
innov = Z - H * X;
A = H * P * H' + R;
chi_lambda = innov' * (A \ innov);
end

function [X, P, innovation, K] = kf_measurement_update(X, P, Z, H, R)
% 扩展卡尔曼滤波器测量更新函数
%
% 输入:
%   X: 状态向量 (N x 1)
%   P: 状态协方差矩阵 (N x N)
%   Z: 测量向量 (M x 1)
%   H: 测量矩阵或测量函数的雅可比矩阵 (M x N)
%   R: 测量噪声协方差矩阵 (M x M)
%
% 输出:
%   X: 更新后的状态向量
%   P: 更新后的状态协方差矩阵
%   innovation: 新息 (M x 1)
%   K: 卡尔曼增益 (N x M)

% 计算新息
innovation = Z - H * X;

% 计算新息协方差
S = H * P * H' + R;

% 计算卡尔曼增益
K = P * H' / S;

% 更新状态向量
X = X + K * innovation;

% 更新状态协方差矩阵
P = (eye(size(P)) - K * H) * P;
end

function [lat, lon, h] = ch_ENU2LLA2(E, N, U, lat0, lon0, h0)

% 经纬高 转 ENU
% lat0, lon0, h0: 起始点经纬高, 经纬度为rad， 高度为m
% lat, lon, h 终点经纬高, 经纬度为rad， 高度为m
% E, N ,U 系下增量，单位为m

%精确算法
% XYZ0 = ch_LLA2ECEF(lat0, lon0, h0);
% XYZ1 = ch_LLA2ECEF(lat, lon, h);
% dXYZ = XYZ1 - XYZ0;
% 
%  [~, ~, C_ECEF2ENU, ~]= ch_earth(lat0, lon0, h0);
%  dENU = C_ECEF2ENU * dXYZ;
%  E = dENU(1);
%  N= dENU(2);
%  U = dENU(3);
 
 %近似算法
R_0 = 6378137; %WGS84 Equatorial radius in meters
clat = cos(lat0);
lon = E / clat / R_0 + lon0;
lat = N / R_0 + lat0;
h = h0 + U;
end
function [yaw ,yawR2D] = yaw_convert_0_2pi(yaw)
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
while(yaw < 0)
    yaw =  yaw + 360 * D2R; 
end
while(yaw >= 360 * D2R)
    yaw = yaw -360 * D2R;
end
yawR2D = R2D * yaw;
end
function [yaw ,yawR2D] = yaw_convert_npi_pi(yaw)
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
while(yaw <= -180 * D2R)
    yaw =  yaw + 360 * D2R; 
end
while(yaw > 180 * D2R)
    yaw = yaw -360 * D2R;
end
yawR2D = R2D * yaw;
end

