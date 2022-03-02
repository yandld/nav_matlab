close all;
clear;
clc;

format long g;
format compact;

dT = 0.01;
deg = 180/pi;
rad = pi/180;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;

load('data20191025.mat');
data_length = length(data);
gyro_data = data(:, 8:10)*rad;
acc_data = data(:, 11:13)*g;
mag_data = data(:, 14:16);
gps_data = [data(:, 20) data(:, 19) data(:, 21)]; %M8P数据
vel_data = data(:, 22:24);
gps_hAcc = data(:, 26);
gps_vAcc = data(:, 27);
gps_sAcc = data(:, 28);
gps_headingAcc = data(:, 29);

%%
lat0 = data(1, 20);
lon0 = data(1, 19);
alt0 = data(1, 21);

Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sind(lat0)*sind(lat0));
Rn = Re * (1 + Earth_e*sind(lat0)*sind(lat0));
Rmh = Rm + alt0;
Rnh = Rn + alt0;

distance_sum = 0;
time_sum = 0;
gps_enu = zeros(data_length, 3);
vel_norm_save = zeros(data_length, 1);
for i=1:data_length
    gps_enu(i,3) = gps_data(i,3) - alt0;
    gps_enu(i,2) = (gps_data(i,1) - lat0) * rad * (Rmh);
    gps_enu(i,1) = (gps_data(i,2) - lon0) * rad * (Rnh) * cosd(lat0);

    vel_norm_save(i) = norm(vel_data(i, :));
    distance_sum = distance_sum + vel_norm_save(i)*dT;
    time_sum = time_sum + 0.01;
end

%%
pitch0 = 0*rad;
roll0 = 0*rad;
yaw0 = 0*rad;
nQb = angle2quat(-yaw0, pitch0, roll0, 'ZXY');

vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(15,1);
P = diag([(5*rad)*ones(1,2), (10*rad), 1*ones(1,3), 10, 10, 10,  (500/3600*rad)*ones(1,3), (10e-3*g)*ones(1,3)])^2;
Q = diag([ones(3,1)*(50/60*rad); ones(3,1)*(0.89); zeros(9,1)])^2;

att_save = zeros(data_length, 3);
vel_save = zeros(data_length, 3);
pos_save = zeros(data_length, 3);
P_save = zeros(data_length, 15);
X_save = zeros(data_length, 15);

tic;
count_sec = 1;
for i=1:data_length
    current_time = toc;
    if current_time>count_sec
        percent = (i)/(data_length);
        clc;
        fprintf('已处理%.3f%%, 用时%.3f秒, 预计还需%.3f秒\n', (i)/(data_length)*100, current_time, current_time/percent*(1-percent));
        count_sec = count_sec + 1;
    end

    %% 捷联更新
    % 单子样等效旋转矢量法
    w_nb_b = gyro_data(i,:)'*rad;
    rotate_vector = w_nb_b*dT;
    rotate_vector_norm = norm(rotate_vector);
    q = [cos(rotate_vector_norm/2); rotate_vector/rotate_vector_norm*sin(rotate_vector_norm/2)]';

    % 姿态更新
    nQb = quatmultiply(nQb, q); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
    nQb = quatnormalize(nQb); %单位化四元数
    bQn = quatinv(nQb); %更新bQn
    nCb = quat2dcm(nQb); %更新nCb阵
    bCn = nCb'; %更新bCn阵

    % 速度更新
    f_b = acc_data(i,:)'*g;
    f_n = quatrotate(bQn, f_b')';
    dv = (f_n + [0; 0; -9.8]); %比力方程
    vel = vel + dv*dT;

    % 位置更新
    pos = pos + vel*dT;

    %% 卡尔曼滤波
    F = zeros(15);
    F(4,2) = -f_n(3); %f_u天向比力
    F(4,3) = f_n(2); %f_n北向比力
    F(5,1) = f_n(3); %f_u天向比力
    F(5,3) = -f_n(1); %f_e东向比力
    F(6,1) = -f_n(2); %f_n北向比力
    F(6,2) = f_n(1); %f_e东向比力
    F(7,4) = 1;
    F(8,5) = 1;
    F(9,6) = 1;
    F(1:3,10:12) = -bCn;
    F(4:6,13:15) = bCn;

    % 状态转移矩阵F离散化
    F = eye(15) + F*dT;

    % 卡尔曼时间更新
    X = F*X;
    P = F*P*F' + Q;

    H = zeros(6,15);
    H(1:3,4:6) = eye(3);
    H(4:6,7:9) = eye(3);

    Z = [vel - vel_data(i,:)'; pos - gps_enu(i,:)'];

    R = diag([0.25*ones(3,1); 5*ones(3,1)])^2;  %M8P参数

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

    % 误差清零
    X(1:9) = zeros(9,1);

%     gyro_bias = X_k(10:12);

    % 信息存储
    [yaw, pitch, roll] = dcm2angle(nCb, 'ZXY');
    yaw = -yaw;
    yaw = yaw + (yaw<0)*2*pi;
    att_save(i,:) = [pitch roll yaw]*deg;
    vel_save(i,:) = vel';
    pos_save(i,:) = pos';
    
    X_save(i, :) = X';
    P_save(i, :) = sqrt(diag(P))';
end

clc;
fprintf('已处理完毕，用时%.3f秒\n', toc);

%%
figure;
scatter(gps_enu(:,1)/1e3, gps_enu(:,2)/1e3, 5, vel_norm_save*3.6, 'filled');
ct = colorbar('southoutside');
ct.Label.String = 'Velocity(km/h)';
caxis([0, 120]); % 速度colorbar范围(0~120km/h)
axis equal; grid on;
xlabel('East(km)');
ylabel('North(km)');
title('2D Trajectory and Velocity');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

% hold on;
% plot(gps_enu(:,1)/1e3, gps_enu(:,2)/1e3);

%%
figure;
plot(pos_save(:,1)/1e3, pos_save(:,2)/1e3); hold on;
plot(gps_enu(:,1)/1e3, gps_enu(:,2)/1e3); hold on;
plot(gps_enu(:,1)/1e3, gps_enu(:,2)/1e3, '.');
axis equal; grid on;
legend('KF', 'GNSS');
xlabel('East(km)');
ylabel('North(km)');
title('2D Trajectory');
set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);

%%
fprintf('行驶距离: %.3fkm\n', distance_sum/1000);
fprintf('行驶时间: %d小时%d分%.2f秒\n', degrees2dms(time_sum/3600));
fprintf('最高时速: %.3fkm/h\n', max(vel_norm_save)*3.6);
