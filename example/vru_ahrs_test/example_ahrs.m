close all;
clear;
clc;


%%

%% Import and plot sensor data
load('imu_dataset.mat');

%%  plot sensor data  gyr单位为dps
imu = dataset.imu;
dt = mean(diff(imu.time));
N = length(imu.time);

quat(:,1) = [1 0 0 0]';
err_state = zeros(6, 1); %失准角(3) , 陀螺零偏(3)


%强制加一个bias测试
imu.gyr(3,:) =  imu.gyr(3,:) + deg2rad(2);

[P, Q] = init_filter(dt);

for i = 1:N
    
    % 陀螺仪零偏反馈
    imu.gyr(:,i)  = imu.gyr(:,i) -  err_state(4:6);
    
    %
    %     imu.gyr(1,i) = deg2rad(1);
    %     imu.gyr(2,i) = deg2rad(2);
    %     imu.gyr(3,i) = deg2rad(3);
    %
    %     imu.acc(1,i) = 0.01;
    %     imu.acc(2,i) = 0.12;
    %     imu.acc(3,i) = 0.98;
    %
    %     imu.mag(1,i) = 0.01;
    %     imu.mag(2,i) = 0.12;
    %     imu.mag(3,i) = 0.98;
    
    %	q = ch_mahony.imu(q, imu.gyr(:,i), imu.acc(:,i), dt, 1);
    %    q = ch_mahony.ahrs(q, imu.gyr(:,i), imu.acc(:,i), imu.mag(:,i), dt, 1);
    
    % 导航方程
    gyr = imu.gyr(:,i);
    acc =  imu.acc(:,i);
    
    quat = ch_att_upt(quat, gyr, dt);
    
    %计算F阵
    [F, G] = state_space_model(quat, dt);
    
    %状态递推
    % err_state = F*err_state;
    
    P = F*P*F' + G*Q*G';
    
    
    
    % 重力量测更新
    [P, quat, err_state]=  measurement_update_gravity(quat, err_state,  acc, P);
    
    %磁量测更新
    %  [P, q, err_state]=  measurement_update_mag(q, err_state,  imu.mag(:,i), P);
    
    %记录历史数据
    outdata.eul(:,i) = ch_q2eul(quat);
	outdata.phi(:,i) = err_state(1:3);
    outdata.gyr_bias(:,i) = err_state(4:6);
    outdata.P(:,:,i) = P;
    
    err_state(1:3) = 0;
end

outdata.eul = rad2deg(outdata.eul);
fprintf("最终姿态角:%f, %f %f\n", outdata.eul(:,end));

%% plot

figure('NumberTitle', 'off', 'Name', '原始数据');
subplot(2, 2, 1);
plot(dataset.imu.acc');
legend("X", "Y", "Z");
title("加速度测量值");
subplot(2, 2, 2);
plot(dataset.imu.gyr');
legend("X", "Y", "Z");
title("陀螺测量值");
subplot(2, 2, 3);
plot(dataset.imu.mag');
legend("X", "Y", "Z");
title("磁场");

figure('NumberTitle', 'off', 'Name', 'KF状态量');
subplot(2, 1, 1);
plot(rad2deg(outdata.gyr_bias'));
legend("X", "Y", "Z");
title("陀螺零偏(deg)");
subplot(2, 1, 2);
plot(rad2deg(outdata.phi'));
legend("X", "Y", "Z");
title("失准角(deg)");

% 记录 陀螺零偏和失准角方差
P_wb = zeros(3, N);
P_phi = zeros(3, N);

for i = 1: length(outdata.P)
    P = outdata.P(:,:,i);
    P_phi(1, i) = P(1,1);
    P_phi(2, i) = P(2,2);
    P_phi(3, i) = P(3,3);
    P_wb(1, i) = P(4,4);
    P_wb(2, i) = P(5,5);
    P_wb(3, i) = P(6,6);
end

figure('NumberTitle', 'off', 'Name', '方差');
subplot(2, 1, 1);
plot(P_phi', ".-");
legend("X", "Y", "Z");
title("失准角方差");
subplot(2, 1, 2);
plot(P_wb', ".-");
legend("X", "Y", "Z");
title("陀螺零偏方差");



% F和G
function [F,G] = state_space_model(x, dt)

Cb2n = ch_q2m(x(1:4));

O = zeros(3);

F = [ O -Cb2n; O O];
%离散化
F = eye(6) + F*dt;
G = eye(6);
end


function [P, Q] = init_filter(dt)

Q_att = 0.1;
Q_wb = 0;

P = diag([0.1*[1 1 1], 0.01*[1 1 1]]);

Q = zeros(6);
Q(1:3,1:3) = Q_att*eye(3);
Q(4:6,4:6) = Q_wb*eye(3);
Q = Q*dt^(2);

end


function [P, q, err_state]= measurement_update_gravity(q, err_state, acc, P)

%量测噪声
R_sigma = 4;
R = zeros(2,2);
R(1:2,1:2) = R_sigma*eye(2);

% 加速度计单位化
acc = acc / norm(acc);    % normalise magnitude

% 建立量测矩阵 严龚敏书 7.5.14
H = ch_askew([0 0 -1]');
H = H(1:2,:);
H = [H zeros(2,3)];

%计算新息
z = ch_qmulv(q, -acc) - [0 0 -1]';

%计算增益
K=(P*H')/(H*P*H'+R);

%更新状态
err_state = err_state +  K*(z(1:2) - H*err_state);

%更新P 使用Joseph 形式，取代 (I-KH)*P, 这么数值运算更稳定
N = 6;
P = (eye(N) - K * H) * P;

%误差状态反馈及误差清零
q = ch_qmul(ch_rv2q(err_state(1:3)), q);
end


% 
% function [P, q,  err_state]= measurement_update_mag(q, err_state, mag, P)
% %地磁量测噪声
% R = eye(3)*2;
% 
% % 磁场单位化
% mag = mag / norm(mag);   % normalise magnitude
% 
% %计算新息
% h = ch_qmulv(q, mag);
% b = [norm([h(1) h(2)]) 0 h(3)]';
% h = h - b;
% 
% % 建立量测矩阵 严龚敏书 7.5.14
% H = ch_askew(b);
% H = [H zeros(3)];
% 
% %计算增益
% K=(P*H')/(H*P*H'+R);
% 
% %更新状态
% err_state = err_state +  K*(h - H*err_state);
% 
% %
% % %更新P 使用Joseph 形式，取代 (I-KH)*P, 这么数值运算更稳定
% % I_KH = (eye(size(P,1)) - K*H);
% % P= I_KH*P*I_KH' + K*R*K';
% 
% P = (eye(6) - K*H)*P;
% 
% %误差状态反馈及误差清零
% 
% %对于地磁，只纠正航向角9
%  err_state(1) = 0;
%  err_state(2) = 0;
% 
% %误差反馈及清0
% q(1:4) = ch_qmul(ch_rv2q(err_state(1:3)), q);
% 
% end


