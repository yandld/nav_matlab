clc; clear; close all;

D2R=pi/180;
g=9.8;
N = 15;
imu_dt = 0.01;
X = zeros(15,1);

od_speed = 2.0;

dT = 0.01;
nQb = [0.533215448243828 0.592817248117098 0.0831095662269988 0.597780725760344]';
vel = [0 0 0]';
pos = [0 0 0]';

% 初始状态方差:    水平姿态           航向       东北天速度      水平位置   高度      陀螺零偏                 加速度计零偏
P = diag([(2*D2R)*ones(1,2), (180*D2R), 0.5*ones(1,3), 5*ones(1,2), 10, (50/3600*D2R)*ones(1,3), (10e-3*g)*ones(1,3)])^2;
% 系统方差:       角度随机游走           速度随机游走
Q = diag([(1/60*D2R)*ones(1,3),  (2/60)*ones(1,3),  zeros(1,9)])^2;



for i=0:99
    w_nb_b = [0.814724,0.905792,0.126987]';
    f_b = [0.546882,0.957507,9.964889]';
    
    % 纯惯性姿态更新
    [nQb, pos, vel, q] = ins(w_nb_b, f_b, nQb, pos, vel, g, dT);
    
    bQn = ch_qconj(nQb); %更新bQn
    f_n = ch_qmulv(nQb, f_b);
    a_n = f_n + [0; 0; -g];
    bCn = ch_q2m(nQb); %更新bCn阵
    nCb = bCn'; %更新nCb阵
    
    g_n = [0 0 -9.8];
    %% 卡尔曼滤波
    if(mod(i, 10) == 0)

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
        F = eye(15) + F*0.1;
        
        % 卡尔曼时间更新
        X = F*X;
        P = F*P*F' + Q*0.1;
    end
    
    
    %% gnss
    H = zeros(6,15);
    H(1:3,4:6) = eye(3);
    H(4:6,7:9) = eye(3);
    
    Z = [vel - [0.1 0.2 0.3]'; pos - [1 2 3]'];
    
    R = diag([0.1*ones(2,1); 0.2; 1*ones(2,1); 2])^2;
    
    % 卡尔曼量测更新
    K = P * H' / (H * P * H' + R);
    X = X + K * (Z - H * X);
    P = (eye(length(X)) - K * H) * P;
    
    %% gra
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
    
    %% zupt
    H = zeros(3, 15);
    H(1:3,4:6) = eye(3);
    
    Z = vel;
    
    R = diag(0.1*ones(1,3))^2;
    
    K = P * H' / (H * P * H' + R);
    X = X + K * (Z - H * X);
    P = (eye(length(X)) - K * H) * P;
    
    
    %nhc
    H = zeros(2,15);
    A = [1 0 0; 0 0 1];
    H(1:2,4:6) = A*nCb;
    Z = 0 + (A*nCb)*vel;
    R = diag(ones(1, size(H, 1))*0.1)^2;
    
    K = P * H' / (H * P * H' + R);
    X = X + K * (Z - H * X);
    P = (eye(N) - K * H) * P;
    
    %od
    H = zeros(1,15);
    A = [0 1 0;];
    H(1,4:6) = A*nCb;
    Z = 0 + (A*nCb)*vel - od_speed;
    R = diag(ones(1, size(H, 1))*0.1)^2;
    
    K = P * H' / (H * P * H' + R);
    X = X + K * (Z - H * X);
    P = (eye(N) - K * H) * P;
    
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
    
end

nQb
vel
pos
