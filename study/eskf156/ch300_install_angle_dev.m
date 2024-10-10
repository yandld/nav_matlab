close all;
clear;
clc;

format long g;
format compact;

N = 17;             % ESKF维度
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.8;      % 重力加速度

%% 相关选项及参数设置
opt.alignment_time = 1;          % 初始对准时间(s)
opt.gnss_outage = 0;             % 模拟GNSS丢失
opt.outage_start = 970;         % 丢失开始时间(s)
opt.outage_stop = 990;          % 丢失结束时间(s)
opt.nhc_enable = 1;              % 车辆运动学约束
opt.nhc_lever_arm = 0*[0.35,0.35,-1.35]; %nhc杆臂长度 b系下（右-前-上）240816测试杆臂,仅测试天向，其他两轴为目测值
opt.nhc_R = 3;                % 车载非完整性约束噪声
opt.gnss_min_interval = 0;    % 设定时间间隔，例如0.5秒
opt.gnss_delay = 0.03;              % GNSS量测延迟 sec
opt.gnss_lever_arm = 0*[0.45;0.45;-1.34]; %GNSS杆臂长度 b系下（右-前-上）240816测试杆臂
opt.has_install_esti = 1;       %% can close or open ;1:esti insatllangle ; 0:no esti

opt.zupt_enable = 1;              % 启用ZUPT
opt.zupt_vel_threshold = 2.5;     % ZUPT速度阈值(m/s)
opt.zupt_gyr_threshold = 0.5*D2R; % ZUPT角速度阈值(rad/s)
opt.zupt_time_threshold = 1.0;    % ZUPT持续时间阈值(s)
opt.zupt_R = diag([0.1, 0.1, 0.1].^2); % ZUPT测量噪声协方差


%%
% Cb2n: b系到n系的旋转矩阵 Vn = Cb2n * Vb

% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

%% 数据载入
load('dataset\241008\241008B5.mat');

%单位国际化
data.imu.acc =  data.imu.acc*GRAVITY;
data.imu.gyr =  data.imu.gyr*D2R;
data.gnss.lat = data.gnss.lat*D2R;
data.gnss.lon = data.gnss.lon*D2R;
data.dev.lat = data.dev.ins_lat*D2R;
data.dev.lon = data.dev.ins_lon*D2R;

%加入仿真噪声
% data.imu.acc(:,2) = data.imu.acc(:,2) + 0.1*GRAVITY;
% data.imu.gyr(:,3) = data.imu.gyr(:,3) + 0.5*D2R;

att = [0 0 0]*D2R; %初始安装角
Cb2v_simulate = att2Cnb(att); % matlab仿真设置的安装角误差
Cb2v = eye(3);

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
zupt_detect_time = 0;
is_zupt = 0;
%% 说明
% KF 状态量: 失准角(3) 速度误差(3) 位置误差(3) 陀螺零偏(3) 加计零偏(3)


% 初始状态方差:    姿态       ENU速度  水平位置      陀螺零偏                加速度计零偏        安装俯仰角 安装航向角
opt.P0 = diag([[2 2 10]*D2R, [1 1 1], [5 5 5], 0.03*D2R*ones(1,3), 0.01*GRAVITY*ones(1,3), 2*D2R*ones(1,2) ])^2;
N = length(opt.P0);
% 系统误差:         角度随机游走          速度随机游走                     角速度随机游走            加速度随机游走
opt.Q = diag([(0.1*D2R)*ones(1,3), (0.01)*ones(1,3), 0*ones(1,3), 2.5/3600*D2R*ones(1,3), 0*GRAVITY*ones(1,3), [0 0] ])^2;

imu_len = length(data.imu.tow);
dev_len = length(data.dev.tow);

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
    if norm(data.gnss.vel_enu(i,:)) > 3
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
Qb2n = angle2quat(-yaw0, pitch0, roll0, 'ZXY');
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(N,1);
gyr_bias = X(10:12);
acc_bias = X(13:15);

P = opt.P0;

log.pitch = zeros(imu_len, 1);
log.roll = zeros(imu_len, 1);
log.yaw = zeros(imu_len, 1);
log.vel = zeros(imu_len, 3);
log.pos = zeros(imu_len, 3);
log.P = zeros(imu_len, N);
log.X = zeros(imu_len, N);
log.gyr_bias = zeros(imu_len, 3);
log.acc_bias = zeros(imu_len, 3);
log.sins_att = zeros(imu_len, 3);
log.vb = zeros(imu_len, 3);
log.installangle = zeros(imu_len, 3);
tic;
last_time = toc;
gnss_idx = inital_gnss_idx;
imucnt= 0;

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

    %% 卡尔曼滤波
    Cb2n = ch_q2m(Qb2n); %更新Cb2n阵
    Cn2b = Cb2n'; %更新Cn2b阵

    [X, P, f_n] = imu_propagation(X, P, f_b, Cb2n, opt.Q, imu_dt);

    a_n = f_n + [0; 0; -GRAVITY];
    log.vb(i, :) = (Cn2b * vel)';

    current_time = data.imu.tow(i);
    gnss_lost_elapsed = current_time - gnss_last_valid_time;

    %% GNSS量测更新
    if gnss_idx <= length(data.gnss.tow) && abs(data.imu.tow(i) - data.gnss.tow(gnss_idx)) < 0.01 % threshold 是允许的最大差异

        if data.gnss.solq_pos(gnss_idx) > 0

            gnss_vel_R = diag(ones(3,1) * data.gnss.gnss_vel_std_n(gnss_idx))^2 * 1;
            gnss_pos_R = diag(ones(3,1) * data.gnss.gnss_pos_std_n(gnss_idx))^2 * 1;

            % gnss_vel_R = diag([0.1 0.1 0.1])^2;
            % gnss_pos_R = diag([1.2 1.2 1.2])^2;

            Hvel = zeros(3,N);
            Hvel(1:3, 4:6) = eye(3);
            Zvel = vel - data.gnss.vel_enu(gnss_idx,:)';
            Zvel = Zvel - a_n*opt.gnss_delay; % GNSS量测延迟补偿
            Zvel = Zvel + (-Cb2n*v3_skew(w_b))*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿

            Hpos = zeros(3,N);
            Hpos(1:3, 7:9) = eye(3);
            Zpos = pos - gnss_enu(gnss_idx,:)';

            Zpos = Zpos - vel*opt.gnss_delay; % GNSS量测延迟补偿
            Zpos = Zpos + (-Cb2n)*opt.gnss_lever_arm; % GNSS天线杆壁效应补偿

            if (opt.gnss_outage == 0 || (opt.gnss_outage == 1 && (current_time < opt.outage_start || current_time > opt.outage_stop))) ...
                    && (current_time - last_gnss_fusion_time >= opt.gnss_min_interval)

                if isfield(data.gnss, 'dual_enu')
                    dual_enu_n = data.gnss.dual_enu(gnss_idx, :);

                    if norm(dual_enu_n) > 0.5 && norm(vel) < 2
                        dual_enu_n = dual_enu_n / norm(dual_enu_n);

                        set_baseline_v = [-1, 0, 0]'; % V系中双天线设定位置
                        [H_dualgnss, Z_dualgnss] = lc_eskf_meas_set_dualgnss(Cb2n, Cb2v, dual_enu_n', set_baseline_v);

                        R_dualgnss = diag([2, 2, 2].^2);  % 根据实际情况调整测量噪声

                        [X, P, ~, ~] = kf_measurement_update(X, P, Z_dualgnss, H_dualgnss, R_dualgnss);

                        FB_BIT = bitor(FB_BIT, ESKF156_FB_A);

                    end
                end

                % 卡方：  Reference:  一种基于软卡方检测的自适应Ｋａｌｍａｎ滤波方法
                [chi_lambda_vel, ~] = calculate_chi_square(X, P, Hvel, Zvel, gnss_vel_R);
                log.lambda_vel(gnss_idx, :) = chi_lambda_vel;

                [chi_lambda_pos, ~] = calculate_chi_square(X, P, Hpos, Zpos, gnss_pos_R);
                log.lambda_pos(gnss_idx, :) = chi_lambda_pos;

                % 计算速度和位置的不确定性
                vel_uncertainty_std = sqrt(norm(P(4:6, 4:6)));
                pos_uncertainty_std = sqrt(norm(P(7:9, 7:9)));

                % 判断是否进行GNSS更新
                update_gnss = false;

                if (chi_lambda_vel < 0.5 && chi_lambda_pos < 0.15) || (data.gnss.gnss_vel_std_n(gnss_idx) < 0.3 && data.gnss.gnss_pos_std_n(gnss_idx) < 5)
                    update_gnss = true;
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_G);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_W);
                elseif gnss_lost_elapsed > 3 && (vel_uncertainty_std > 1.0 || pos_uncertainty_std > 20)
                    update_gnss = true;
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
                    fprintf('GNSS被迫更新(%.f秒): 丢失时间 %.2f秒, 速度不确定性: %.2f, 位置不确定性: %.2f\n', ...
                        current_time, gnss_lost_elapsed, vel_uncertainty_std, pos_uncertainty_std);
                end

                % 执行GNSS更新
                if update_gnss
                    [X, P, ~, ~] = kf_measurement_update(X, P, Zvel, Hvel, gnss_vel_R);
                    [X, P, ~, ~] = kf_measurement_update(X, P, Zpos, Hpos, gnss_pos_R);
                    gnss_last_valid_time = data.gnss.tow(gnss_idx);
                end

                % 假设失准角对应的状态量索引为1:3，加速度计零偏对应的状态量索引为13:15
                acc_bias_converged = all(diag(P(13:15, 13:15)) < (0.003*GRAVITY)^2);
                gyr_bias_converged = all(diag(P(17:17, 17:17)) < (0.6*D2R)^2);

                % 确保 FB_BIT 是无符号整数类型
                FB_BIT = uint32(FB_BIT);

                % 只有当加速度计零偏收敛时，才反馈加速度计零偏
                if acc_bias_converged
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_G));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_G))));
                end

                if gyr_bias_converged
                    FB_BIT = bitor(FB_BIT, uint32(ESKF156_FB_W));
                else
                    FB_BIT = bitand(FB_BIT, uint32(bitcmp(uint32(ESKF156_FB_W))));
                end

                % 更新上一次融合的时间
                last_gnss_fusion_time = current_time;
            end
        end
        gnss_idx = gnss_idx + 1;
    end

    %% NHC 约束
    if(opt.nhc_enable)
        norm_vel = norm(vel);

        if norm_vel > 0.2 && norm(w_b) < 20*D2R
            if gnss_lost_elapsed > 3
                H = zeros(2,N);
                A_pos = [1 0 0; 0 0 1];
                H(:,4:6) = A_pos*Cb2v*Cn2b;
                Z = 0 + (A_pos*Cb2v*Cn2b)*vel;
                R = diag(ones(1, size(H, 1))*opt.nhc_R/1.5 + norm_vel/1.5)^2;


                [X, P, ~, ~] = kf_measurement_update(X, P, Z, H, R);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_P);

            else
                %% v frame 速度误差作为安装角估计的量测方程 [Vvx_ins,Vvy_ins,Vvz_ins]=dVv = Vv'-Vv = -Cb2v*Cn2b*v3_skew(Vn)*dphi + v3_skew(Cb2v*Cn2b*Vn)*dinstallangle + Cb2v*Cn2b*dVn
                %% paper ref:车载MIMUGNSS组合高精度无缝导航关键技术研究--sunzhenqian
                if (~opt.has_install_esti)
                    Cb2v = eye(3);
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

                Vn2v_sins = Cb2v * log.vb(i, 1:3)';

                Z = Vn2v_sins([1,3]);

                R = diag(ones(1, size(H, 1))*opt.nhc_R)^2;

                [X, P, ~, ~] = kf_measurement_update(X, P, Z, H, R);

                FB_BIT = bitor(FB_BIT, ESKF156_FB_A);
                FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
                if (opt.has_install_esti)
                    FB_BIT = bitor(FB_BIT, ESKF156_FB_CBV);
                end
            end
        end
    end

    %% ZUPT检测和更新
    if opt.zupt_enable
        vel_norm = norm(vel);
        gyr_norm = norm(w_b);

        if vel_norm < opt.zupt_vel_threshold && gyr_norm < opt.zupt_gyr_threshold
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
            zupt_detect_time = 0;

            H = zeros(3, N);
            H(:, 4:6) = eye(3);  % 速度误差状态

            z_zupt = vel;  % 观测值：当前速度（应该接近零）

            [X, P, ~, ~] = kf_measurement_update(X, P, z_zupt, H, opt.zupt_R);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_V);
            FB_BIT = bitor(FB_BIT, ESKF156_FB_P);
        end
    end

    %% 反馈
    if bitand(FB_BIT, ESKF156_FB_A)
        rv = X(1:3);
        rv_norm = norm(rv);
        if rv_norm > 0
            qe = [cos(rv_norm/2); sin(rv_norm/2)*rv/rv_norm]';
            Qb2n = ch_qmul(qe, Qb2n);
            Qb2n = ch_qnormlz(Qb2n); %单位化四元数
            Cb2n = ch_q2m(Qb2n); %更新Cb2n阵
            Cn2b = Cb2n'; %更新Cn2b阵
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
        X(16:17) = 0;
        att = m2att(Cb2v);
    end



    %% 信息存储
    [pitch, roll, yaw] = q2att(Qb2n);
    log.pitch(i,:) = pitch;
    log.roll(i,:) = roll;
    log.yaw(i,:) = yaw;
    log.installangle(i,:)=att'*180/pi;
    log.vel(i,:) = vel';
    log.pos(i,:) = pos';
    log.X(i, :) = X';
    log.P(i, :) = sqrt(diag(P))';
    log.gyr_bias(i, :) = gyr_bias;
    log.acc_bias(i, :) = acc_bias;
    log.tow(i,:) = current_time;
end

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
    'Layout', [2, 2], ...
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

%% 2D
navigation_plots.trajectory_2d_plot({gnss_enu, log.pos, dev_pos_enu}, {'GNSS', 'MATLAB', 'DEV嵌入式设备轨迹'});


%% 数据统计
time_duration = seconds(time_sum);
time_duration.Format = 'hh:mm:ss';

fprintf('行驶时间: %s\n', char(time_duration));
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('最高时速: %.3fkm/h\n', max(log.vel_norm)*3.6);


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


function [H, Z] = lc_eskf_meas_set_dualgnss(Cb2n, Cb2v, bl_vec, set_baseline_v)
% 计算V系中的基线向量
set_bl_v = set_baseline_v;
set_bl_v = set_bl_v / norm(set_bl_v);

Cv2b = Cb2v';
set_bl_b = Cv2b*set_bl_v;


% 将基线向量从B系转换到N系
set_bl_n = Cb2n * set_bl_b;

% 处理GNSS提供的基线向量（已经在N系中）
mes_bl_n = bl_vec / norm(bl_vec);

% 计算测量残差
Z = set_bl_n - mes_bl_n;

% 设置测量矩阵H
H = zeros(3, 17);  % N是状态向量的维度
bl_skew = skew(bl_vec);
H(:, 1:3) = bl_skew;
end

function S = skew(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

