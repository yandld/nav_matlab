clc
clear
close all

%% 说明
% UWB IMU 融合算法，采用误差卡尔曼15维经典模型，伪距组合

% noimal_state:     位置(3) 速度(3) 四元数(4) 共10维
% err_state:           位置误差(3) 速度误差(3) 失准角(3) 加速度计零偏(3) 陀螺零偏(3) 共15维
% du:                    加速度计零偏(3) 陀螺零偏(3)

%% Motion Process, Measurement model and it's derivative
h_func = @uwb_h;
dh_dx_func = @err_uwb_h;


%% load data set
%load dataset2;
load datas;
dataset = datas;

N = length(dataset.imu.time);
dt = sum(diff(dataset.imu.time)) / N;

dataset.uwb.cnt = 4;

uwb_noise = 0.25;  % UWB测距噪声

R = diag(ones(dataset.uwb.cnt, 1)*uwb_noise^(2));
p_div = 0; % 预测频率器，目前没有用
m_div = 0; %量测分频器

%% 打印原始数据
ch_plot_imu('time', 1:length(dataset.imu.acc), 'acc', dataset.imu.acc' / 9.8, 'gyr', rad2deg(dataset.imu.gyr'));

%% out data init
out_data.uwb = [];
out_data.uwb.time = dataset.uwb.time;
out_data.imu.time = dataset.imu.time;
out_data.uwb.anchor = dataset.uwb.anchor;

%% load settings
settings = uwb_imu_example_settings();
noimal_state = init_navigation_state(settings);
du = zeros(6, 1);
[P, Q1, Q2, ~, ~] = init_filter(settings);

fprintf("共%d数据, 频率:%d Hz\n", N,  1 / dt);

fprintf("开始融合...\n");
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
    
    [pos, vel, q] = ch_nav_equ_local_tan(pos, vel, q, acc, gyr, dt, [0, 0, 9.8]');
    
    noimal_state(1:3) = pos;
    noimal_state(4:6) = vel;
    noimal_state(7:10) = q;
    
    
    p_div = p_div+1;
    if p_div == 1
        % 生成F阵   G阵
        [F, G] = state_space_model(noimal_state, acc, dt*p_div);
        
        %卡尔曼P阵预测公式
        P = F*P*F' + G*blkdiag(Q1, Q2)*G';
        p_div = 0;
    end
    
    
    
    %% EKF UWB量测更新
    m_div = m_div+1;
     if m_div == 1
        m_div = 0;
        
        pr = dataset.uwb.tof(:,k);
        
        % bypass Nan
        if sum(isnan(pr)) == 0
            [~,H] = dh_dx_func(noimal_state, dataset.uwb);
            
            % 卡尔曼公式，计算K
            K = (P*H')/(H*P*H'+R);
            
            % NLOS elimation
            % t = h_func(noimal_state, dataset.uwb);
            %             if uwb_iter > 50
            %                 for i = 1:length(y)
            %                     if abs(y(i) - t(i))  > 0.9
            %                         y(i) = t(i); %丢弃这次量测，直接认为这次量测就是预测误差
            %                     end
            %                 end
            %             end
            
            err_state = [zeros(9,1); du] + K*(pr - h_func(noimal_state, dataset.uwb));
            
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
    
    out_data.x(k,:)  = noimal_state;
    out_data.delta_u(k,:) = du';
    out_data.diag_P(k,:) = trace(P);
end

%% 纯 UWB 位置解算
j = 1;
uwb_pos = [0 0 0]';
N = length(dataset.uwb.time);

for i=1:N
    pr = dataset.uwb.tof(:, i);
    % 去除NaN点
    if all(~isnan(pr)) == true
        
        uwb_pos = ch_multilateration(dataset.uwb.anchor, uwb_pos,  pr');
        out_data.uwb.pos(:,j) = uwb_pos;
        j = j+1;
    end
end


%% plot 数据
out_data.uwb.tof = dataset.uwb.tof;
out_data.uwb.fusion_pos = out_data.x(:,1:3)';

% fusion_display(out_data, []);
 
 
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
subplot(2,1,1);
plot(out_data.x(:,1:3));
legend("X", "Y", "Z");
title("位置");
subplot(2,1,2);
plot(out_data.x(:,4:6));
legend("X", "Y", "Z");
title("速度");


figure;
plot(out_data.uwb.pos(1,:), out_data.uwb.pos(2,:), '.');
hold on;
plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '*-');
legend("伪距解算UWB轨迹", "融合轨迹");

figure;
plot(datas.pos(1,:), datas.pos(2,:), '.');
hold on;
plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '*-');
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


