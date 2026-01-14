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
opt.outage_start = 30000;         % 丢失开始时间(s)
opt.outage_stop = 40000;          % 丢失结束时间(s)
opt.nhc_enable = 1;              % 车辆运动学约束
opt.nhc_lever_arm = 0*[0.40;0.00;0.55]; %nhc杆臂长度 b系下（右-前-上）
opt.nhc_R = 1;                % 车载非完整性约束噪声
opt.gnss_min_interval = 0;    % 设定时间间隔，例如0.5秒
opt.gnss_delay = 0.000;              % GNSS量测延迟 sec
opt.gnss_lever_arm = 1*[-0.04;0.0;0.4]; %GNSS杆臂长度 b系下（右-前-上）
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

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

%% 数据载入
load("dataset\HI83-246-024126\HI83-246-024126.mat");

data.gnss.gnss_delay = data.gnss.gnss_delay;
data.imu.tow = data.imu.tow - 0.000;

%单位国际化
data.imu.acc =  data.imu.acc*GRAVITY;
data.imu.gyr =  data.imu.gyr*D2R;
data.gnss.lat = data.gnss.lat*D2R;
data.gnss.lon = data.gnss.lon*D2R;
data.dev.lat = data.dev.ins_lat*D2R;
data.dev.lon = data.dev.ins_lon*D2R;
data.dev.cali_status = opt.cali_status;

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

%% KF参数设置
opt.Q = zeros(N,N);
opt.P0 = zeros(N,N);
opt.P1 =  zeros(N,N);
opt.P0 = diag([[2 2 10]*D2R, [1 1 1], [5 5 5], 0.11*D2R*[1,1,1], 0.001*GRAVITY*ones(1,3), 1*D2R*[2 20],1.0*[1,1,0.1]])^2;
opt.P1 = diag([[2 2 10]*D2R, [1 1 1], [5 5 5], 0.11*D2R*[1,1,1], 0.001*GRAVITY*[1,1,1], 0*D2R*[2 20] ,0.0*[1,1,1]])^2;
opt.Q = diag([(0.05*D2R)*[1,1,1], (0.01)*ones(1,3), 0*ones(1,3), 2.0/3600*D2R*[1,1,0.1], 25/1e6*GRAVITY*ones(1,3), 0.0001/3600*[1 10]*D2R , 0.00001/3600*[1,1,1]])^2;

imu_len = length(data.imu.tow);
dev_len = length(data.dev.tow);
gnss_len = length(data.gnss.tow);
imu_dt = mean(diff(data.imu.tow));
gnss_dt = mean(diff(data.gnss.tow));

% 开启静止时间
indices = find(data.imu.tow <= opt.alignment_time);
acc_align0 = data.imu.acc(indices, :);
gyr_align0 = data.imu.gyr(indices, :);
gyr_bias0 = mean(gyr_align0);

% 结束静止时间
start_tow = data.imu.tow(end) - opt.alignment_time;
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

last_gnss_fusion_time = -inf;

yaw_flag = zeros(imu_len,1);
init_yaw_count = 0;
inital_gnss_idx = 0;

% 根据速度获得初始航向角
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

%% 一阶kf滤波参数
install_yaw_dual2V_filter = 0;
P_DUAL2V = 100 * D2R;
Q_DUAL2V = 0 * D2R;
R_DUAL2V = 10 * D2R;
od_scale_factor_P = 0.5;
od_scale_factor_Q = 0;
od_scale_factor_R = 0.005;
vel_norm0 = 0;
err = [0 0 0]';

%% 初始化航向
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
        
        norm_f_b = f_b/norm(f_b);
        Qn2b = Qb2n .* [1 -1 -1 -1];
        gb = ch_qmulv(Qn2b, gn)';
        err = cross(norm_f_b , gb);
        a_n = ch_qmulv(Qb2n, f_b);
        log.err(i,:) =err';
    end
   
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

    % GNSS数据获取初始速度
    if (inital_gnss_idx+1) <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(inital_gnss_idx+1)) < 0.01
        inital_gnss_idx = inital_gnss_idx + 1;
        if norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1 
            init_yaw_count = init_yaw_count +1;
            opt.inital_yaw = atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2));
            opt.inital_yaw = yaw_convert_0_2pi(opt.inital_yaw);
            
            % 根据速度和双天线航向估计双天线安装角度
            if data.gnss.solq_heading(inital_gnss_idx,1) == 4
                 dual_heading = data.gnss.dual_heading_deg(inital_gnss_idx) * D2R; % 直接使用GNSS航向角(deg转rad)
                 vel_heading = atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2));
                 install_yaw_dual2V = (vel_heading - dual_heading + opt.anta *D2R);
                 
                 [install_yaw_dual2V,~] = yaw_convert_npi_pi(install_yaw_dual2V);
                 if install_yaw_dual2V_filter == 0 && install_yaw_dual2V < 30 * D2R
                     install_yaw_dual2V_filter = install_yaw_dual2V;
                 end
                 if (norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1) && install_yaw_dual2V < 30 * D2R
                    [install_yaw_dual2V_filter, P_DUAL2V, ~, ~] = kf_measurement_update(install_yaw_dual2V_filter, P_DUAL2V, install_yaw_dual2V, 1, R_DUAL2V);
                 end 
            end
            log.install_yaw_dual2V_filter(inital_gnss_idx,1) = install_yaw_dual2V_filter;
            
            % 单天线初始化航向
            if(init_yaw_count >opt.init_yaw_time/0.1 ) || (norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 3 && 0.5*init_yaw_count >opt.init_yaw_time/0.1)
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                
                fprintf("单天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                fprintf("速度估计IMU安装角:%.2f°, v系初始航向:%f\r\n",  R2D * yaw_convert_0_2pi(atan2(vel0(1,i),vel0(2,i))-yaw00(i)*D2R), atan2(data.gnss.vel_enu(inital_gnss_idx,1),data.gnss.vel_enu(inital_gnss_idx,2))*R2D);
                fprintf("启动IMU速度：%f,%f",vel0(1,i),vel0(2,i));
                break;
            % 双天线初始化航向
            elseif data.gnss.solq_heading(inital_gnss_idx,1) == 4 && opt.use_dualgnss_flag
                
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                opt.inital_yaw = data.gnss.dual_heading_deg(inital_gnss_idx) * D2R; % 直接使用GNSS航向角
                [yaw_diff , ~ ] = yaw_convert_npi_pi(opt.inital_yaw - (vel_heading + opt.anta*D2R));
                if norm(data.gnss.vel_enu(inital_gnss_idx,:)) > 1 &&  abs(yaw_diff)<30 * D2R
                    opt.inital_yaw = yaw_convert_0_2pi(opt.inital_yaw - opt.anta*D2R - (atan2(vel0(1,i),vel0(2,i)) - yaw00(i)*D2R));
                    fprintf("双天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                    break;
                end
           
            end
        elseif data.gnss.solq_heading(inital_gnss_idx,1) == 4 && opt.use_dualgnss_flag && i > 1
                
                diff_values = abs(data.imu.tow - data.gnss.tow(inital_gnss_idx));
                [~, inital_imu_idx] = min(diff_values);
                [opt.inital_yaw, ~] = yaw_convert_0_2pi(data.gnss.dual_heading_deg(inital_gnss_idx) * D2R); % 直接使用GNSS航向角
                fprintf("双天线启动初始速度航向角:%.2f°, 从GNSS数据:%d开始, IMU数据:%d\r\n",  opt.inital_yaw*R2D, inital_gnss_idx, i);
                break; 
        end
    
    end
end
initial_id = i;

% 航向初始化失败
if i == length(data.gnss.tow)
    opt.inital_yaw = 0;
    fprintf("无法通过速度矢量找到初始航向角，设置为:%.2f°\r\n",  opt.inital_yaw*R2D);
    inital_imu_idx = 1;
end

% 位置速度转换
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
pitch0 = asin(-g_b(2));
roll0 = atan2( g_b(1), -g_b(3));
yaw0 = opt.inital_yaw;
Qb2n = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(N,1);
gyr_bias = gyr_bias0';
acc_bias = X(13:15);

if data.dev.cali_status == 0
    data.dev.cali_status = 1;
    P = opt.P0;
elseif data.dev.cali_status == 2
    P = opt.P1;
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
log.lambda_vel = zeros(gnss_len, 1);
log.lambda_pos = zeros(gnss_len, 1);
log.gnss_vel_chi = zeros(gnss_len, 1);
log.gnss_pos_chi = zeros(gnss_len, 1);
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

%% 主循环
for i=inital_imu_idx:imu_len
    imucnt=imucnt+1;
    FB_BIT = 0;
    curr_time = toc;
    if curr_time - last_time >= 1
        fprintf('已完成 %.2f%%\n', (i / imu_len) * 100);
        last_time = curr_time;
    end

    %% 捷联更新
    w_b = data.imu.gyr(i,:)';
    w_b = Cb2v_simulate*w_b;
    w_b = w_b - gyr_bias;

    f_b = data.imu.acc(i,:)';
    f_b = Cb2v_simulate*f_b;
    f_b = f_b - acc_bias;

    [Qb2n, pos, vel, ~] = inertial_navigation_update(w_b, f_b, Qb2n, pos, vel, GRAVITY, imu_dt);
    [~, ~, yaw] = q2att(Qb2n);

    %% 卡尔曼滤波
    Cb2n = ch_q2m(Qb2n);
    Cn2b = Cb2n';

    [X, P, f_n] = imu_propagation(X, P, f_b, Cb2n, opt.Q, imu_dt);

    a_n = f_n + [0; 0; -GRAVITY];
    log.vb(i, :) = (Cn2b * vel)';

    current_time = data.imu.tow(i);
    gnss_lost_elapsed = current_time - gnss_last_valid_time;

    %% GNSS量测更新
    if gnss_idx <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(gnss_idx)) < 0.01
        
        % 根据速度和双天线航向估计双天线安装角度
        if data.gnss.solq_heading(gnss_idx,1) == 4  && norm(data.gnss.vel_enu(gnss_idx))>1
             dual_heading = data.gnss.dual_heading_deg(gnss_idx) * D2R; % 直接使用GNSS航向角
             vel_heading = atan2(data.gnss.vel_enu(gnss_idx,1),data.gnss.vel_enu(gnss_idx,2));
             while dual_heading < 0
                dual_heading = dual_heading +360*D2R;
             end
             install_yaw_dual2V = (vel_heading - dual_heading + opt.anta*D2R);
             [install_yaw_dual2V,~] = yaw_convert_npi_pi(install_yaw_dual2V);
             if (norm(data.gnss.vel_enu(gnss_idx,:)) > 1) && install_yaw_dual2V < 30 * D2R
                [install_yaw_dual2V_filter, P_DUAL2V, ~, ~] = kf_measurement_update(install_yaw_dual2V_filter, P_DUAL2V, install_yaw_dual2V, 1, R_DUAL2V);
             end 
        end
        
        update_gnss = false;
        if data.gnss.solq_pos(gnss_idx) > 0

            gnss_vel_n = norm(data.gnss.vel_enu(gnss_idx, 1:3));
            loggnss.solq = data.gnss.solq_pos(gnss_idx);
            Hvel = zeros(3,N);
            Hvel(1:3, 4:6) = eye(3);
            Zvel = vel - data.gnss.vel_enu(gnss_idx,:)';
            Zvel = Zvel - a_n * (opt.gnss_delay + data.gnss.gnss_delay(gnss_idx)/1000 );
            Zvel = Zvel + (Cb2n*v3_skew(w_b))*opt.gnss_lever_arm;

            Hpos = zeros(3,N);
            Hpos(1:3, 7:9) = eye(3);
            Zpos = pos - gnss_enu(gnss_idx,:)';
            
            Zpos = Zpos - vel*(opt.gnss_delay + data.gnss.gnss_delay(gnss_idx)/1000 ) + 0.0 * 0.04*0.04 * a_n;
            Zpos = Zpos + (Cb2n)*opt.gnss_lever_arm;
            
            if N==20
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
            
            X_pre = X(a);
            
            if (opt.gnss_outage == 0 || (opt.gnss_outage == 1 && (current_time < opt.outage_start || current_time > opt.outage_stop))) && (current_time - last_gnss_fusion_time >= opt.gnss_min_interval) 

                if isfield(data.gnss, 'dual_heading_deg')   
                    
                    if data.gnss.solq_heading(gnss_idx,1) == 4
                        
                        if norm(data.gnss.vel_enu(gnss_idx, 1:2))>1 && data.dev.cali_status < 3 && 0
                            dual_heading = data.gnss.dual_heading_deg(gnss_idx) * D2R;
                            Z_angle_gv = vel_heading - dual_heading;
                            [install_yaw_dual2V, P_dual_angle, ~, ~] = kf_measurement_update(install_yaw_dual2V, P_dual_angle+Q_dual_angle, Z_angle_gv, 1, R_dual_angle);
                        end
                        
                        [~,dual_heading] = yaw_convert_0_2pi(data.gnss.dual_heading_deg(gnss_idx) * D2R);
                        
                        Z_dual_heading = yaw  + (att(3)*R2D) + opt.anta - (dual_heading + (install_yaw_dual2V_filter) *R2D + w_b(3)*(opt.gnss_delay+data.gnss.gnss_delay(gnss_idx)/1000 - 0.015) * R2D );
                        [~,Z_dual_heading] = yaw_convert_npi_pi(Z_dual_heading*D2R);
                        
                        R_dual_heading = (0.5 * D2R).^2;
                        H_dual_heading = zeros(1,N);
                        H_dual_heading(1,3) = 1;
                        H_dual_heading(1,17) = 1;

                        if data.dev.cali_status > 0 && norm(w_b)<3*D2R && norm(data.gnss.vel_enu(gnss_idx, 1:2))<1.1 && 1
                            [X, P, ~, ~] = kf_measurement_update(X, P, Z_dual_heading*D2R, H_dual_heading, R_dual_heading);
                            X_pre = X(a);
                            FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                            X(16:17) = 0;
                        end
                    end
                end

                [chi_lambda_vel, ~] = calculate_chi_square(X, P, Hvel, Zvel, gnss_vel_R);
                log.lambda_vel(gnss_idx, :) = sqrt(chi_lambda_vel);

                [chi_lambda_pos, ~] = calculate_chi_square(X, P, Hpos, Zpos, gnss_pos_R);
                log.lambda_pos(gnss_idx, :) = sqrt(chi_lambda_pos);
                log.gnss_vel_chi(gnss_idx,:) = data.gnss.gnss_vel_std_n(gnss_idx)^2 * chi_lambda_vel;
                log.gnss_pos_chi(gnss_idx,:) = data.gnss.gnss_pos_std_n(gnss_idx)^2 * chi_lambda_pos;

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
                
                if update_gnss && (opt.gnss_outage == 0 || (current_time < opt.outage_start || current_time > opt.outage_stop))
                    if norm(data.gnss.vel_enu(gnss_idx,:))>0.3
                        [X, P, ~, ~] = kf_measurement_update(X, P, Zvel, Hvel, gnss_vel_R);
                        X_pre = X(a);
                    end
                    
                    [X, P, ~, ~] = kf_measurement_update(X, P, Zpos, Hpos, gnss_pos_R);
                    X_pre = X(a);
                    gnss_last_valid_time = data.gnss.tow(gnss_idx);
                end
             end

                acc_bias_converged = all(diag(P(13:14, 13:14)) < (0.008 * GRAVITY)^2);
                gyr_bias_converged = 1;
                lever_arm_converged = gyr_bias_converged;
                FB_BIT = uint32(FB_BIT);

                if acc_bias_converged  || 1
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_G));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_G))));
                    X(13:15) = 0;
                end

                if gyr_bias_converged
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_W));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_W))));
                    X(10:12) = 0;
                end

                if lever_arm_converged ||1
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_LEVER));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_LEVER))));
                    X(18:20) = 0;
                end
                
                last_gnss_fusion_time = current_time;
        end
        gnss_idx = gnss_idx + 1;
    
    end

    %% NHC 约束
    if(opt.nhc_enable && 1)
        norm_vel = norm(vel);
        
        if (norm_vel > 0.1  && (gnss_lost_elapsed >= 1 && norm(w_b) < 200*D2R)) || (gnss_lost_elapsed < 1 && norm(w_b) < 20*D2R && norm_vel > 0.5)
            if (~opt.has_install_esti)
                Cb2v = eye(3);
            end
            
            Vn2v_sins = Cb2v * log.vb(i, 1:3)' + (Cb2n*v3_skew(w_b))*opt.nhc_lever_arm;
            
            if Vn2v_sins(2) < -1 && norm(Vn2v_sins)>3
                angle = atan2(Vn2v_sins(1),Vn2v_sins(2));
                Cb2v = Cb2v * att2Cnb([0 0 pi]);
                Vn2v_sins = Cb2v * log.vb(i, 1:3)';
            end
            
            if Vn2v_sins(2)< -0.2 
                reverse_heading_flag = 1;
            elseif Vn2v_sins(2)> -0.01
                reverse_heading_flag = 0;
            end
            
            M1 = - Cb2v * Cn2b * v3_skew(vel);
            M2 = Cb2v * Cn2b;
            M3 = v3_skew(Cb2v * log.vb(i,:)');
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
            if (opt.has_install_esti)  && norm(w_b) < 3*D2R && update_gnss
                FB_BIT = bitor(FB_BIT, ESKF156_FB_CBV);
            else
                X(16) = 0;
                X(17) = 0;
            end
        end
    end
    log.X_INSTALL(i,:) = X(16:17)'*R2D;
    
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
            is_zupt = false;
            if gnss_lost_elapsed < 1 &&  gnss_vel_n<0.3
                H = zeros(3, N);
                H(:, 4:6) = eye(3);
                z_zupt = vel;
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zupt, H, opt.zupt_R);
                
                H = zeros(3, N);
                H(:, 10:12) = eye(3);
                z_zaru = w_b;
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zaru, H, opt.zaru_R);
    
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
            elseif gnss_lost_elapsed > 1
                H = zeros(3, N);
                H(:, 4:6) = eye(3);
                z_zupt = vel;
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zupt, H, opt.zupt_R);
                
                H = zeros(3, N);
                H(:, 10:12) = eye(3);
                z_zaru = w_b;
                [X, P, ~, ~] = kf_measurement_update(X, P, z_zaru, H, opt.zaru_R);
    
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
            end
        end
    end
    
    if data.dev.cali_status <2
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
            Qb2n = ch_qnormlz(Qb2n);
            Cb2n = ch_q2m(Qb2n);
            Cn2b = Cb2n';
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
    [pitch, roll, yaw] = q2att(Qb2n);
    log.pitch(i,:) = pitch;
    log.roll(i,:) = roll;
    log.yaw(i,:) = yaw;
    log.installangle(i,:)=att'*180/pi;
    log.vel(i,:) = vel';
    log.Z_vel(i,:) = Zvel;
    log.pos(i,:) = pos';
    log.Z_pos(i,:) = Zpos;
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
end

vel0=vel0';
pos0=pos0';
fprintf('数据处理完毕，用时%.3f秒\n', toc);

set(groot, 'defaultAxesXGrid', 'on');
set(groot, 'defaultAxesYGrid', 'on');
set(groot, 'defaultAxesZGrid', 'on');

%% ==================== 全新绘图部分 ====================

%% 1. 轨迹对比图 (2D和3D)
figure('Name', '轨迹对比', 'Position', [100, 100, 1400, 600]);

% 2D轨迹
subplot(1,2,1);
plot(gnss_enu(:,1), gnss_enu(:,2), 'r.-', 'LineWidth', 1.5, 'MarkerSize', 4);
hold on;
plot(log.pos(:,1), log.pos(:,2), 'b-', 'LineWidth', 2);
plot(dev_pos_enu(:,1), dev_pos_enu(:,2), 'g--', 'LineWidth', 1.5);
xlabel('东向 (m)');
ylabel('北向 (m)');
title('2D轨迹对比');
legend('GNSS', 'MATLAB', 'HI30');
grid on;
axis equal;

% 3D轨迹
subplot(1,2,2);
plot3(gnss_enu(:,1), gnss_enu(:,2), gnss_enu(:,3), 'r.-', 'LineWidth', 1.5, 'MarkerSize', 4);
hold on;
plot3(log.pos(:,1), log.pos(:,2), log.pos(:,3), 'b-', 'LineWidth', 2);
plot3(dev_pos_enu(:,1), dev_pos_enu(:,2), dev_pos_enu(:,3), 'g--', 'LineWidth', 1.5);
xlabel('东向 (m)');
ylabel('北向 (m)');
zlabel('高程 (m)');
title('3D轨迹对比');
legend('GNSS', 'MATLAB', 'HI30');
grid on;
view(45, 30);

%% 2. 姿态对比
figure('Name', '姿态对比', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, log.roll, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.roll, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('横滚角 (°)');
title('Roll角对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,2);
plot(data.imu.tow, log.pitch, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.pitch, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('俯仰角 (°)');
title('Pitch角对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,3);
plot(data.imu.tow, log.yaw, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.yaw, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('航向角 (°)');
title('Yaw角对比');
legend('MATLAB', 'HI30');
grid on;

%% 3. 速度对比
figure('Name', '速度对比', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, log.vel(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.vel_enu(:,1), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('东向速度 (m/s)');
title('东向速度对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,2);
plot(data.imu.tow, log.vel(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.vel_enu(:,2), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('北向速度 (m/s)');
title('北向速度对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,3);
plot(data.imu.tow, log.vel(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.vel_enu(:,3), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('天向速度 (m/s)');
title('天向速度对比');
legend('MATLAB', 'HI30');
grid on;

%% 4. 位置对比
figure('Name', '位置对比', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, log.pos(:,1), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, dev_pos_enu(:,1), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('东向位置 (m)');
title('东向位置对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,2);
plot(data.imu.tow, log.pos(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, dev_pos_enu(:,2), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('北向位置 (m)');
title('北向位置对比');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,3);
plot(data.imu.tow, log.pos(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, dev_pos_enu(:,3), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('高程 (m)');
title('高程对比');
legend('MATLAB', 'HI30');
grid on;

%% 5. 陀螺零偏
figure('Name', '陀螺零偏', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, log.gyr_bias(:,1)*R2D*3600, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_wb(:,1)*R2D*3600, 'r--', 'LineWidth', 1.5);
yline(gyr_bias0(1)*R2D*3600, 'k-.', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('X轴陀螺零偏 (°/h)');
title('X轴陀螺零偏');
legend('MATLAB', 'HI30', '初始估计');
grid on;

subplot(3,1,2);
plot(data.imu.tow, log.gyr_bias(:,2)*R2D*3600, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_wb(:,2)*R2D*3600, 'r--', 'LineWidth', 1.5);
yline(gyr_bias0(2)*R2D*3600, 'k-.', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('Y轴陀螺零偏 (°/h)');
title('Y轴陀螺零偏');
legend('MATLAB', 'HI30', '初始估计');
grid on;

subplot(3,1,3);
plot(data.imu.tow, log.gyr_bias(:,3)*R2D*3600, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_wb(:,3)*R2D*3600, 'r--', 'LineWidth', 1.5);
yline(gyr_bias0(3)*R2D*3600, 'k-.', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('Z轴陀螺零偏 (°/h)');
title('Z轴陀螺零偏');
legend('MATLAB', 'HI30', '初始估计');
grid on;

%% 6. 加速度计零偏
figure('Name', '加速度计零偏', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, log.acc_bias(:,1)*1000/GRAVITY, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_gb(:,1)*1000/GRAVITY, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('X轴加计零偏 (mG)');
title('X轴加速度计零偏');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,2);
plot(data.imu.tow, log.acc_bias(:,2)*1000/GRAVITY, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_gb(:,2)*1000/GRAVITY, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('Y轴加计零偏 (mG)');
title('Y轴加速度计零偏');
legend('MATLAB', 'HI30');
grid on;

subplot(3,1,3);
plot(data.imu.tow, log.acc_bias(:,3)*1000/GRAVITY, 'b-', 'LineWidth', 1.5);
hold on;
plot(data.imu.tow, data.dev.kf_gb(:,3)*1000/GRAVITY, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('Z轴加计零偏 (mG)');
title('Z轴加速度计零偏');
legend('MATLAB', 'HI30');
grid on;

%% 7. 安装角估计
figure('Name', '安装角估计', 'Position', [100, 100, 1400, 600]);

subplot(1,2,1);
plot(data.imu.tow, log.installangle(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('俯仰安装角 (°)');
title('俯仰安装角估计');
grid on;

subplot(1,2,2);
plot(data.imu.tow, log.installangle(:,3), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('航向安装角 (°)');
title('航向安装角估计');
grid on;

%% 8. GNSS质量指标
figure('Name', 'GNSS质量指标', 'Position', [100, 100, 1400, 900]);

subplot(3,2,1);
plot(data.gnss.tow, data.gnss.gnss_vel_std_n, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度标准差 (m/s)');
title('GNSS速度标准差');
grid on;

subplot(3,2,2);
plot(data.gnss.tow, data.gnss.gnss_pos_std_n, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置标准差 (m)');
title('GNSS位置标准差');
grid on;

subplot(3,2,3);
plot(data.gnss.tow, log.lambda_vel, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度卡方检验');
title('速度卡方检验');
grid on;

subplot(3,2,4);
plot(data.gnss.tow, log.lambda_pos, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置卡方检验');
title('位置卡方检验');
grid on;

subplot(3,2,5);
plot(data.gnss.tow, data.gnss.nv, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('卫星数');
title('可见卫星数量');
grid on;

subplot(3,2,6);
plot(data.gnss.tow, data.gnss.solq_pos, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('定位质量');
title('GNSS定位质量');
grid on;
ylim([0, 5]);

%% 9. 双天线信息
if isfield(data.gnss, 'dual_heading_deg')
    figure('Name', '双天线信息', 'Position', [100, 100, 1400, 900]);
    
    subplot(3,1,1);
    plot(data.gnss.tow, data.gnss.dual_base_len, 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('基线长度 (m)');
    title('双天线基线长度');
    grid on;
    
    subplot(3,1,2);
    plot(data.gnss.tow, data.gnss.dual_pitch_deg, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(data.imu.tow, data.dev.pitch, 'r--', 'LineWidth', 1.5);
    plot(data.imu.tow, log.pitch, 'g-.', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('俯仰角 (°)');
    title('俯仰角对比');
    legend('双天线俯仰', 'HI30俯仰', 'MATLAB俯仰');
    grid on;
    
    subplot(3,1,3);
    plot(data.gnss.tow, data.gnss.dual_heading_deg, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(data.imu.tow, data.dev.yaw, 'r--', 'LineWidth', 1.5);
    plot(data.imu.tow, log.yaw, 'g-.', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('航向角 (°)');
    title('航向角对比');
    legend('双天线航向', 'HI30航向', 'MATLAB航向');
    grid on;
end

%% 10. 状态量误差
figure('Name', 'KF状态量', 'Position', [100, 100, 1400, 900]);

subplot(3,3,1);
plot(data.imu.tow, log.X(:,1)*R2D, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('失准角 (°)');
title('俯仰失准角');
grid on;

subplot(3,3,2);
plot(data.imu.tow, log.X(:,2)*R2D, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('失准角 (°)');
title('横滚失准角');
grid on;

subplot(3,3,3);
plot(data.imu.tow, log.X(:,3)*R2D, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('失准角 (°)');
title('航向失准角');
grid on;

subplot(3,3,4);
plot(data.imu.tow, log.X(:,4), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度误差 (m/s)');
title('东向速度误差');
grid on;

subplot(3,3,5);
plot(data.imu.tow, log.X(:,5), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度误差 (m/s)');
title('北向速度误差');
grid on;

subplot(3,3,6);
plot(data.imu.tow, log.X(:,6), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('速度误差 (m/s)');
title('天向速度误差');
grid on;

subplot(3,3,7);
plot(data.imu.tow, log.X(:,7), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('东向位置误差');
grid on;

subplot(3,3,8);
plot(data.imu.tow, log.X(:,8), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('北向位置误差');
grid on;

subplot(3,3,9);
plot(data.imu.tow, log.X(:,9), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置误差 (m)');
title('高程误差');
grid on;

%% 11. 杆臂和时间延迟估计
if N == 20
    figure('Name', '杆臂和时间延迟', 'Position', [100, 100, 1400, 600]);
    
    subplot(1,3,1);
    plot(data.imu.tow, log.gnss_lever_arm(:,1), 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('杆臂X (m)');
    title('GNSS杆臂X估计');
    grid on;
    
    subplot(1,3,2);
    plot(data.imu.tow, log.gnss_lever_arm(:,2), 'r-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('杆臂Y (m)');
    title('GNSS杆臂Y估计');
    grid on;
    
    subplot(1,3,3);
    plot(data.imu.tow, log.gnss_delay, 'g-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('时间延迟 (s)');
    title('IMU/GNSS时间延迟估计');
    grid on;
end

%% 12. NHC约束信息
figure('Name', 'NHC约束', 'Position', [100, 100, 1400, 600]);

subplot(1,3,1);
plot(data.imu.tow, log.Vn2v_sins(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('车体系X速度 (m/s)');
title('车体系横向速度');
grid on;
yline(0, 'r--', 'LineWidth', 1);

subplot(1,3,2);
plot(data.imu.tow, log.Vn2v_sins(:,2), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('车体系Y速度 (m/s)');
title('车体系前向速度');
grid on;

subplot(1,3,3);
plot(data.imu.tow, log.Vn2v_sins(:,3), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('车体系Z速度 (m/s)');
title('车体系天向速度');
grid on;
yline(0, 'r--', 'LineWidth', 1);

%% 13. GNSS更新标志
figure('Name', 'GNSS更新状态', 'Position', [100, 100, 1400, 400]);

subplot(1,2,1);
plot(data.imu.tow, log.is_gnss_update, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('更新标志');
title('GNSS更新标志');
grid on;
ylim([-0.1, 1.1]);

subplot(1,2,2);
plot(data.imu.tow, log.reverse_heading_flag, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('倒车标志');
title('倒车检测标志');
grid on;
ylim([-0.1, 1.1]);

%% 14. 位置误差分析
pos_error = log.pos - dev_pos_enu;
figure('Name', '位置误差分析', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, pos_error(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('东向误差 (m)');
title('MATLAB vs HI30 东向位置误差');
grid on;

subplot(3,1,2);
plot(data.imu.tow, pos_error(:,2), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('北向误差 (m)');
title('MATLAB vs HI30 北向位置误差');
grid on;

subplot(3,1,3);
plot(data.imu.tow, pos_error(:,3), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('高程误差 (m)');
title('MATLAB vs HI30 高程误差');
grid on;

%% 15. 速度误差分析
vel_error = log.vel - data.dev.vel_enu;
figure('Name', '速度误差分析', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, vel_error(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('东向速度误差 (m/s)');
title('MATLAB vs HI30 东向速度误差');
grid on;

subplot(3,1,2);
plot(data.imu.tow, vel_error(:,2), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('北向速度误差 (m/s)');
title('MATLAB vs HI30 北向速度误差');
grid on;

subplot(3,1,3);
plot(data.imu.tow, vel_error(:,3), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('天向速度误差 (m/s)');
title('MATLAB vs HI30 天向速度误差');
grid on;

%% 16. 姿态误差分析
yaw_error = arrayfun(@(i) angle_difference(log.yaw(i), data.dev.yaw(i)), 1:length(log.yaw))';
att_error = [log.roll - data.dev.roll, log.pitch - data.dev.pitch, yaw_error];

figure('Name', '姿态误差分析', 'Position', [100, 100, 1400, 900]);

subplot(3,1,1);
plot(data.imu.tow, att_error(:,1), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('横滚误差 (°)');
title('MATLAB vs HI30 横滚角误差');
grid on;

subplot(3,1,2);
plot(data.imu.tow, att_error(:,2), 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('俯仰误差 (°)');
title('MATLAB vs HI30 俯仰角误差');
grid on;

subplot(3,1,3);
plot(data.imu.tow, att_error(:,3), 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('航向误差 (°)');
title('MATLAB vs HI30 航向角误差');
grid on;
ylim([-180, 180]);
yline(0, 'k--', 'LineWidth', 1);

%% 17. 统计信息输出
fprintf('\n========== 统计信息 ==========\n');
time_duration = seconds(time_sum);
time_duration.Format = 'hh:mm:ss';
fprintf('行驶时间: %s\n', char(time_duration));
fprintf('行驶距离: %.3f km\n', distance_sum/1000);
fprintf('最高时速: %.3f km/h\n', max(log.vel_norm)*3.6);
fprintf('平均速度: %.3f km/h\n', (distance_sum/time_sum)*3.6);

fprintf('\n位置误差统计 (MATLAB vs HI30):\n');
fprintf('  东向RMS: %.3f m\n', rms(pos_error(:,1)));
fprintf('  北向RMS: %.3f m\n', rms(pos_error(:,2)));
fprintf('  高程RMS: %.3f m\n', rms(pos_error(:,3)));
fprintf('  水平RMS: %.3f m\n', rms(sqrt(pos_error(:,1).^2 + pos_error(:,2).^2)));

fprintf('\n速度误差统计 (MATLAB vs HI30):\n');
fprintf('  东向RMS: %.4f m/s\n', rms(vel_error(:,1)));
fprintf('  北向RMS: %.4f m/s\n', rms(vel_error(:,2)));
fprintf('  天向RMS: %.4f m/s\n', rms(vel_error(:,3)));

fprintf('\n姿态误差统计 (MATLAB vs HI30):\n');
fprintf('  横滚RMS: %.4f °\n', rms(att_error(:,1)));
fprintf('  俯仰RMS: %.4f °\n', rms(att_error(:,2)));
fprintf('  航向RMS: %.4f °\n', rms(att_error(:,3)));
fprintf('  航向最大误差: %.4f °\n', max(abs(att_error(:,3))));
fprintf('  航向平均误差: %.4f °\n', mean(att_error(:,3)));
fprintf('  航向标准差: %.4f °\n', std(att_error(:,3)));

fprintf('\n陀螺零偏最终估计:\n');
fprintf('  X: %.3f °/h\n', log.gyr_bias(end,1)*R2D*3600);
fprintf('  Y: %.3f °/h\n', log.gyr_bias(end,2)*R2D*3600);
fprintf('  Z: %.3f °/h\n', log.gyr_bias(end,3)*R2D*3600);

fprintf('\n加速度计零偏最终估计:\n');
fprintf('  X: %.3f mG\n', log.acc_bias(end,1)*1000/GRAVITY);
fprintf('  Y: %.3f mG\n', log.acc_bias(end,2)*1000/GRAVITY);
fprintf('  Z: %.3f mG\n', log.acc_bias(end,3)*1000/GRAVITY);

fprintf('\n安装角最终估计:\n');
fprintf('  俯仰: %.3f °\n', log.installangle(end,1));
fprintf('  横滚: %.3f °\n', log.installangle(end,2));
fprintf('  航向: %.3f °\n', log.installangle(end,3));

fprintf('==============================\n');

%% ==================== 辅助函数 ====================

function angle_diff = angle_difference(angle1, angle2)
    % 计算两个角度的最小差值（考虑周期性）
    diff = angle1 - angle2;
    diff = mod(diff + 180, 360) - 180;
    angle_diff = diff;
end

function [chi_lambda, innov] = calculate_chi_square(X, P, H, Z, R)
    innov = Z - H * X;
    A = H * P * H' + R;
    chi_lambda = innov' * (A \ innov);
end

function [X, P, innovation, K] = kf_measurement_update(X, P, Z, H, R)
    innovation = Z - H * X;
    S = H * P * H' + R;
    K = P * H' / S;
    X = X + K * innovation;
    P = (eye(size(P)) - K * H) * P;
end

function [yaw, yawR2D] = yaw_convert_0_2pi(yaw)
    R2D = 180/pi;
    D2R = pi/180;
    while(yaw < 0)
        yaw = yaw + 360 * D2R; 
    end
    while(yaw >= 360 * D2R)
        yaw = yaw - 360 * D2R;
    end
    yawR2D = R2D * yaw;
end

function [yaw, yawR2D] = yaw_convert_npi_pi(yaw)
    R2D = 180/pi;
    D2R = pi/180;
    while(yaw <= -180 * D2R)
        yaw = yaw + 360 * D2R; 
    end
    while(yaw > 180 * D2R)
        yaw = yaw - 360 * D2R;
    end
    yawR2D = R2D * yaw;
end

