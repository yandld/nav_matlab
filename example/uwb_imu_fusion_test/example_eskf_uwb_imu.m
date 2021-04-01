clc
clear
close all

%% 说明
% UWB IMU 融合算法，采用误差卡尔曼15维经典模型，伪距组合

% noimal_state:     位置(3) 速度(3) 四元数(4) 共10维
% err_state:           位置误差(3) 速度误差(3) 失准角(3) 加速度计零偏(3) 陀螺零偏(3) 共15维
% du:                    加速度计零偏(3) 陀螺零偏(3)

%% load data set
load datas3;
dataset = datas;

N = length(dataset.imu.time);
dt = sum(diff(dataset.imu.time)) / N;

dataset.uwb.cnt = 4;
%dataset.uwb.anchor = [dataset.uwb.anchor; [0.88 0.87 0.87 0.87]];

uwb_noise = 0.05;  % UWB测距噪声

R = diag(ones(dataset.uwb.cnt, 1)*uwb_noise^(2));
p_div_cntr = 0; % 预测频率器，目前没有用
m_div_cntr = 0; %量测分频器
m_div = 10;  %每m_div次量测，才更新一次KF, 节约计算量或者做实验看效果


%% out data init
out_data.uwb = [];
out_data.uwb.time = dataset.uwb.time;
out_data.imu.time = dataset.imu.time;
out_data.uwb.anchor = dataset.uwb.anchor;

%% load settings
settings = uwb_imu_example_settings();
noimal_state = init_navigation_state(settings);
err_state = zeros(15, 1);

%使用第一帧伪距作为初始状态
pr = dataset.uwb.tof(:, 1);
noimal_state(1:3) = ch_multilateration(dataset.uwb.anchor, [ 0 0 0]',  pr', 3);

du = zeros(6, 1);
[P, Q1, Q2, ~, ~] = init_filter(settings);

fprintf("共%d帧数据, 采样频率:%d Hz 共运行时间 %d s\n", N,  1 / dt, N * dt);
fprintf("开始滤波...\n");
for k=1:N
    
    acc = dataset.imu.acc(:,k);
    gyr = dataset.imu.gyr(:,k);
    
    % 反馈
    acc = acc + du(1:3);
    gyr = gyr + du(4:6);
    
    % 捷联惯导
    pos = noimal_state(1:3);
    
    vel = noimal_state(4:6);
    q =  noimal_state(7:10);
    
    [pos, vel, q] = ch_nav_equ_local_tan(pos, vel, q, acc, gyr, dt, [0, 0, -9.8]'); % 东北天坐标系，重力为-9.8
    
    %Z方向速度限制
%    vel(3) = 0;
    
    noimal_state(1:3) = pos;
    noimal_state(4:6) = vel;
    noimal_state(7:10) = q;
    out_data.eul(k,:) = ch_q2eul(q);
    
    p_div_cntr = p_div_cntr+1;
    if p_div_cntr == 1
        % 生成F阵   G阵
        [F, G] = state_space_model(noimal_state, acc, dt*p_div_cntr);
        
        %卡尔曼P阵预测公式
        P = F*P*F' + G*blkdiag(Q1, Q2)*G';
        p_div_cntr = 0;
    end
    
    
    
    %% EKF UWB量测更新
    m_div_cntr = m_div_cntr+1;
    if m_div_cntr == m_div
        m_div_cntr = 0;
        
        
         pr = dataset.uwb.tof(:,k);
        %pr = mean(dataset.uwb.tof(:,k-5 : k),2);
        
        % bypass Nan
        if sum(isnan(pr)) == 0
            [Y, H]  = uwb_hx(noimal_state, dataset.uwb, 2);
            
            % 卡尔曼公式，计算K
            K = (P*H')/(H*P*H'+R);
            
            err_state = [zeros(9,1); du] + K*(pr - Y);
            
            % 反馈速度位置
            noimal_state(1:6) = noimal_state(1:6) + err_state(1:6);
            
            % 反馈姿态
            q = noimal_state(7:10);
            q = ch_qmul(ch_qconj(q), ch_rv2q(err_state(7:9)));
            q = ch_qconj(q);
            noimal_state(7:10) = q;
            
            %存储加速度计零偏，陀螺零偏
            du = err_state(10:15);
            
            % P阵后验更新
            P = (eye(15)-K*H)*P;
            
        end
    end
    
    
       % Z轴速度约束
        R2 = eye(1)*0.5;
        Cn2b = ch_q2m(ch_qconj(noimal_state(7:10)));
    
        H = [zeros(1,3), [0 0 1]* Cn2b, zeros(1,9)];
    
        K = (P*H')/(H*P*H'+R2);
        z = Cn2b*noimal_state(4:6);
    
    	err_state = [zeros(9,1); du] + K*(0-z(3:3));
    
        % 反馈速度位置
        noimal_state(1:6) = noimal_state(1:6) + err_state(1:6);
    
        % 反馈姿态
        q = noimal_state(7:10);
        q = ch_qmul(ch_qconj(q), ch_rv2q(err_state(7:9)));
        q = ch_qconj(q);
        noimal_state(7:10) = q;
    
        %存储加速度计零偏，陀螺零偏
      % du = err_state(10:15);
    
       % P阵后验更新
       P = (eye(15)-K*H)*P;
    
    
    out_data.x(k,:)  = noimal_state;
    out_data.delta_u(k,:) = du';
    out_data.diag_P(k,:) = trace(P);
end

fprintf("开始纯UWB最小二乘位置解算...\n");
%% 纯 UWB 位置解算
j = 1;
uwb_pos = [0 0 0]';
N = length(dataset.uwb.time);

for i=1:N
    pr = dataset.uwb.tof(:, i);
    % 去除NaN点
    if all(~isnan(pr)) == true
        
        uwb_pos = ch_multilateration(dataset.uwb.anchor, uwb_pos,  pr', 3);
        out_data.uwb.pos(:,j) = uwb_pos;
        j = j+1;
    end
end
fprintf("计算完成...\n");

%% plot 数据
out_data.uwb.tof = dataset.uwb.tof;
out_data.uwb.fusion_pos = out_data.x(:,1:3)';


%% 打印原始数据
figure;
subplot(2, 2, 1);
plot(dataset.imu.acc');
legend("X", "Y", "Z");
title("加速度测量值");
subplot(2, 2, 2);
plot(dataset.imu.gyr');
legend("X", "Y", "Z");
title("陀螺测量值");
subplot(2, 2, 3);
plot(dataset.uwb.tof');
title("UWB测量值(伪距)");



figure;
subplot(2,1,1);
plot(out_data.delta_u(:,1:3));
legend("X", "Y", "Z");
title("加速度零偏");
subplot(2,1,2);
plot(rad2deg(out_data.delta_u(:,4:6)));
legend("X", "Y", "Z");
title("陀螺仪零偏");

figure;
subplot(2,2,1);
plot(out_data.x(:,1:3));
legend("X", "Y", "Z");
title("位置");
subplot(2,2,2);
plot(out_data.x(:,4:6));
legend("X", "Y", "Z");
title("速度");
subplot(2,2,3);
plot(out_data.eul);
legend("X", "Y", "Z");
title("欧拉角(姿态)");


figure;
subplot(1,2,1);
plot3(out_data.uwb.pos(1,:), out_data.uwb.pos(2,:), out_data.uwb.pos(3,:), '.');
axis equal
title("UWB 伪距解算位置");
subplot(1,2,2);
plot3(datas.pos(1,:), datas.pos(2,:), datas.pos(3,:), '.');
hold on;
plot3(out_data.x(:,1), out_data.x(:,2), out_data.x(:,3), '.-');
axis equal
title("硬件给出轨迹");


figure;
plot(out_data.uwb.pos(1,:), out_data.uwb.pos(2,:), '.');
hold on;
plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '.-');
legend("伪距解算UWB轨迹", "融合轨迹");

figure;
plot(datas.pos(1,:), datas.pos(2,:), '.');
hold on;
plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '.-');
legend("硬件给出轨迹", "融合轨迹");



%%  Init navigation state
function x = init_navigation_state(~)

% 初始化normial state
q = ch_eul2q(deg2rad([0 0 0]));
x = [zeros(6,1); q];
end


%% 初始化滤波器参数
function [P, Q1, Q2, R, H] = init_filter(settings)

% Kalman filter state matrix
P = zeros(15);
P(1:3,1:3) = settings.factp(1)^2*eye(3);
P(4:6,4:6) = settings.factp(2)^2*eye(3);
P(7:9,7:9) = diag(settings.factp(3:5)).^2;
P(10:12,10:12) = settings.factp(6)^2*eye(3);
P(13:15,13:15) = settings.factp(7)^2*eye(3);

% Process noise covariance
Q1 = zeros(6);
Q1(1:3,1:3) = diag(settings.sigma_acc).^2*eye(3);
Q1(4:6,4:6) = diag(settings.sigma_gyro).^2*eye(3);

Q2 = zeros(6);
Q2(1:3,1:3) = settings.sigma_acc_bias^2*eye(3);
Q2(4:6,4:6) = settings.sigma_gyro_bias^2*eye(3);

R =0;
H = 0;

end

%%  生成F阵，G阵
function [F,G] = state_space_model(x, acc, dt)
Cb2n = ch_q2m(x(7:10));

% Transform measured force to force in the tangent plane coordinate system.
sf = Cb2n * acc;
sk = ch_askew(sf);

% Only the standard errors included
O = zeros(3);
I = eye(3);
F = [
    O I   O O       O;
    O O sk Cb2n O;
    O O O O       -Cb2n;
    O O O O       O;
    O O O O       O];

% Approximation of the discret  time state transition matrix
F = eye(15) + dt*F;

% Noise gain matrix
G=dt*[
    O       O         O  O;
    Cb2n  O         O  O;
    O        -Cb2n O  O;
    O        O         I   O;
    O        O        O   I];
end

%% UWB量测过程
% Y 根据当前状态和UWB基站坐标预测出来的伪距
% H 量测矩阵
% dim:  2: 二维  3：三维
function [Y, H] = uwb_hx(x, uwb, dim)

pos = x(1:3);
if(dim)== 2
    pos = pos(1:2);
    uwb.anchor = uwb.anchor(1:2, :);
  %  uwb.anchor
end
N = uwb.cnt;
Y = [];
H = [];
residual = repmat(pos,1,N) - uwb.anchor(:,1:N);
for i=1:N
    
    if(dim)== 2
        H = [H ;residual(:,i)'/norm(residual(:,i)),zeros(1,13)];
    else
        H = [H ;residual(:,i)'/norm(residual(:,i)),zeros(1,12)];
    end
    Y = [Y ;norm(residual(:,i))];
end



end
