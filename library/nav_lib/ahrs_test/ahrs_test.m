%% Start of script
close all;
clear;
clc;

%% Import and plot sensor data
load('imu_dataset.mat');

%%  plot sensor data 
imu = dataset.imu;
dt = mean(diff(imu.time));
n = length(imu.time);

q(:,1) = [1 0 0 0]';
err_state = zeros(6, 1);%失准角， 陀螺误差
wb = [0 0 0]'; %陀螺零偏
[P, Q] = init_filter(dt);


for i = 1:n
    
    
    %强制加一个bias : 11 dps
%     imu.gyr(2,i) =  imu.gyr(2,i) + deg2rad(20);
    imu.gyr(1,i) =  imu.gyr(1,i) - deg2rad(10);
 
     % 陀螺仪零偏反馈
    imu.gyr(:,i)  = imu.gyr(:,i) - 0;
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
    q = ch_qintg(q, imu.gyr(:,i), dt);
    
    %计算F阵
	[F, G] = state_space_model(q, dt);
     
    %状态递推
    err_state = F*err_state;
    
    %误差传播
    P = F*P*F' + G*Q*G';
    

     % 重力量测更新
    [P, q, err_state]=  measurement_update_gravity(q, err_state,  imu.acc(:,i), P);

    %磁量测更新
    [P, q, err_state]=  measurement_update_mag(q, err_state,  imu.mag(:,i), P);

    %P阵强制正定
    P = (P + P')/2;
    
    %记录估计处理的零偏
    wb = err_state(4:6);

    outdata.eul(:,i) = ch_q2eul(q);
    outdata.wb(:,i) = wb;
end

ch_imu_data_plot('acc', imu.acc', 'gyr', imu.gyr', 'mag', imu.mag', 'time', imu.time');

figure('Name', 'Euler Angles');
hold on;
plot(imu.time, outdata.eul(1,:), 'r');
plot(imu.time, outdata.eul(2,:), 'g');
plot(imu.time, outdata.eul(3,:), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

rad2deg( outdata.eul(:,end))

figure;
plot(rad2deg(outdata.wb'));
title('零偏');

% F和G
function [F,G] = state_space_model(x, dt)

Cb2n = ch_q2m(x(1:4));

I = eye(3);
O = zeros(3);

F = [ O -Cb2n; O O];
%离散化
F = eye(6) + F*dt;

G = eye(6);
end


function [P, Q] = init_filter(dt)

Q_att = 2;
Q_wb = 1;

P = eye(6)*2;

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
	err_state = err_state +  K*(z(1:2));

    %更新P
    P=(eye(6)-K*H)*P;
    
    %误差状态反馈及误差清零
    %对于重力矢量量测，只纠正俯仰横滚
  %  err_state(3) = 0;
    q = ch_qmul(ch_rv2q(err_state(1:3)), q);
    err_state(1:3) = 0;
end



function [P, q,  err_state]= measurement_update_mag(q, err_state, mag, P)
%地磁量测噪声
R = eye(3)*2;

% 磁场单位化
mag = mag / norm(mag);   % normalise magnitude

%计算新息
h = ch_qmulv(q, mag);
b = [norm([h(1) h(2)]) 0 h(3)]';
h = h - b;

% 建立量测矩阵 严龚敏书 7.5.14
H = ch_askew(b);
H = [H zeros(3)];

%计算增益
K=(P*H')/(H*P*H'+R);

%更新状态
err_state = err_state +  K*(h);

%更新P
P = (eye(6)-K*H)*P;

%误差状态反馈及误差清零

%对于地磁，只纠正航向角
% err_state(1) = 0;
% err_state(2) = 0;
q(1:4) = ch_qmul(ch_rv2q(err_state(1:3)), q);
err_state(1:3) = 0;

end


