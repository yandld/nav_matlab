close all;
clear;
clc;

data_length = 1e3;

dT = 0.01;
deg = 180/pi;
rad = pi/180;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;

lat0 = 0;
lon0 = 0;
alt0 = 0;

Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
Rn = Re * (1 + Earth_e*sin(lat0)*sin(lat0));
Rmh = Rm + alt0;
Rnh = Rn + alt0;

nQb = [1 0 0 0];
vel = [0 0 0]';
pos = [0 0 0]';

X = zeros(15,1);
P = diag([(2.5*rad)*ones(1,2), (10*rad), 1*ones(1,3), 10, 10, 10,  (500/3600*rad)*ones(1,3), (10e-3*g)*ones(1,3)])^2;
Q = diag([ones(3,1)*(50/60*rad); ones(3,1)*(0.89); zeros(9,1)])^2;

att_save = zeros(data_length, 3);
vel_save = zeros(data_length, 3);
pos_save = zeros(data_length, 3);
P_save = zeros(data_length, 15);
X_save = zeros(data_length, 15);
for i=1:data_length
    w_nb_b = [0.1 -0.2 -0.3]';

    % 单子样等效旋转矢量法
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
    f_b = [0.1 -0.2 10]';
    f_n = quatrotate(bQn, f_b')';
    dv = (f_n + [0; 0; -9.8]); %比力方程
    vel = vel + dv*dT;

    % 位置更新
    pos = pos + vel*dT;

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

    pos_enu = pos2enu(0, 0, 0, 0, 0, 0, Rmh, Rnh);
    Z = [vel - vel; pos - pos_enu];

    R = diag([0.1*ones(3,1); 5*ones(3,1)])^2;  %M8P参数

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
    pos_save(i,:) = pos';;
    
    X_save(i, :) = X';
    P_save(i, :) = sqrt(diag(P))';
end

[yaw, pitch, roll] = quat2angle(nQb, 'ZXY');

function enu = pos2enu(lat, lon, alt, lat0, lon0, alt0, Rmh, Rnh)
    enu = zeros(3,1);
    enu(3) = alt - alt0;
    enu(2) = (lat - lat0) * (Rmh);
    enu(1) = (lon - lon0) * (Rnh) * cos(lat0);
end
