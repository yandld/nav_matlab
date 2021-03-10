clear;
clc;
close all;
%% 当地切平面坐标系 GPS IMU 组合导航，经典15维ESKF

%% Load data 瑞典data
load('gps_ins_dataset1.mat');

%%  load couersera data
%load('gps_ins_dataset2.mat');


%% extract data
u = [dataset.imu.acc; dataset.imu.gyr];
gnss = dataset.gnss.pos_ned;
gnss_time = dataset.gnss.time;
imu_t = dataset.imu.time;


%% load settings
settings = gnss_imu_local_tan_example_settings();

%% Run the GNSS-aided INS
disp('Runs the GNSS-aided INS')
predic_div = 0;
x = init_navigation_state(u, settings);

% Initialize the sensor bias estimate
delta_u = zeros(6, 1);

% Initialize the Kalman filter
[P, Q, ~, ~] = init_filter(settings);

n = size(u,2);

ctr_gnss_data = 1;

for k=2:n
    dt = imu_t(k)-imu_t(k-1);
    
    % 陀螺零偏，人为在陀螺Y轴上加一个大的bias
    u(5,k) = u(5,k) + deg2rad(5);
    
    % 零偏状态反馈
    u_h = u(:,k) - delta_u;
    
    % 捷联惯导解算
    [x(1:3), x(4:6), x(7:10)] = ch_nav_equ_local_tan(x(1:3), x(4:6), x(7:10), u_h(1:3), u_h(4:6), dt, settings.gravity);
    
    predic_div = predic_div+1;
    if predic_div == 10
        
        % Get state space model matrices
        [F, G] = state_space_model(x, u_h, dt*predic_div);
        
        % 方差递推
        P = F*P*F' + G*Q*G';
        
        predic_div = 0;
    end
    
    
    % 量测更新
    if abs(imu_t(k) - gnss_time(ctr_gnss_data)) < 0.01
        if imu_t(k)<settings.outagestart || imu_t(k) > settings.outagestop || ~strcmp(settings.gnss_outage, 'on')
            
            y = gnss(:, ctr_gnss_data);
            H = [eye(3) zeros(3,12)];
            R = [settings.sigma_gps^2*eye(3)];
            
            % Calculate the Kalman filter gain.
            K=(P*H')/(H*P*H'+R);
            
            z = [zeros(9,1); delta_u] + K*(y - x(1:3));
            
            %% Correct the navigation states using current perturbation estimates.
            
            % 位置速度反馈
            x(1:6) = x(1:6) + z(1:6);
            
            % 失准角反馈到姿态
            q = x(7:10);
            q = ch_qmul(ch_rv2q(z(7:9)), q);
            x(7:10) = q;
            
            delta_u = z(10:15);
            
            %更新P 使用Joseph 形式，取代 (I-KH)*P, 这么数值运算更稳定
            I_KH = (eye(size(P,1))-K*H);
            P= I_KH*P*I_KH' + K*R*K';
            
        end
        ctr_gnss_data = min(ctr_gnss_data+1, length(gnss_time));
    end
    
    % Save the data to the output data structure
    outdata.x(:,k) = x;
    outdata.eul(:,k) = ch_q2eul(x(7:10));
    
    outdata.diag_P(:,k) = diag(P);
    outdata.delta_u_h(:,k) = delta_u;
end


%% 绘图
outdata.eul = rad2deg(outdata.eul);
imu = dataset.imu;
imu.gyr = rad2deg(imu.gyr);

% 原始数据
ch_plot_imu('acc', imu.acc', 'gyr', imu.gyr',  'eul', outdata.eul', 'time',  imu.time');

% 零偏估计plot
ch_plot_imu('wb', rad2deg(outdata.delta_u_h(4:6,:))', 'gb', outdata.delta_u_h(1:3,:)', 'time',  imu.time');

% P阵方差
ch_plot_imu('P_phi', outdata.diag_P(7:9,:)', 'P_wb', outdata.diag_P(10:12,:)', 'P_pos', outdata.diag_P(1:3,:)', 'time',  imu.time');

% 2D轨迹
ch_plot_gps_imu_pos('pos',outdata.x(1:3,:)', 'gnss', dataset.gnss.pos_ned',  'time',  imu.time');

% % 3D轨迹
ch_plot_pos3d('p1', outdata.x(1:3,:)', 'p2', dataset.gnss.pos_ned', 'legend', ["融合", "GNSS"] )

figure;
xest = outdata.x(2,:);
yest = outdata.x(1,:);
xgps = interp1(dataset.gnss.time, dataset.gnss.pos_ned(2,:), dataset.imu.time,'linear','extrap')';
ygps = interp1(dataset.gnss.time, dataset.gnss.pos_ned(1,:), dataset.imu.time,'linear','extrap')';
xerr = xest - xgps; 
yerr = yest - ygps;
plot(dataset.imu.time, xerr)
grid on
hold on
plot(dataset.imu.time, yerr)
xlabel('time [s]')
ylabel('position difference [m]')
title('融合后与GNSS位置误差');
legend('x', 'y')

positionerr_RMS = sqrt(mean(xerr.^2+yerr.^2));

fprintf("融合后轨迹与GPS误差:%.3f\n", positionerr_RMS);

%%  Init navigation state     %%
function x = init_navigation_state(~, settings)

roll = 0;
pitch = 0;

% Initial coordinate rotation matrix
q = ch_eul2q([roll pitch settings.init_heading]);
x = [zeros(6,1); q];

end

%%  Init filter
function [P, Q, R, H] = init_filter(settings)


% Kalman filter state matrix
P = zeros(15);
P(1:3,1:3) = settings.factp(1)^2*eye(3);
P(4:6,4:6) = settings.factp(2)^2*eye(3);
P(7:9,7:9) = diag(settings.factp(3:5)).^2;
P(10:12,10:12) = settings.factp(6)^2*eye(3);
P(13:15,13:15) = settings.factp(7)^2*eye(3);

% Process noise covariance
Q = zeros(12);
Q(1:3,1:3) = diag(settings.sigma_acc).^2*eye(3);
Q(4:6,4:6) = diag(settings.sigma_gyro).^2*eye(3);
Q(7:9,7:9) = settings.sigma_acc_bias^2*eye(3);
Q(10:12,10:12) = settings.sigma_gyro_bias^2*eye(3);

% GNSS-receiver position measurement noise
R = settings.sigma_gps^2*eye(3);

% Observation matrix
H = [eye(3) zeros(3,12)];

end


%%  State transition matrix
function [F, G] = state_space_model(x, u, dt)
Cb2n = ch_q2m(x(7:10));

% Transform measured force to force in the tangent plane coordinate system.
sf = Cb2n * u(1:3);
sk = ch_askew(sf);

% Only the standard errors included
O = zeros(3);
I = eye(3);
F = [  O I   O O        O;
    O O -sk -Cb2n O;
    O O O O       -Cb2n;
    O O O O       O;
    O O O O       O];

% 离散化
F = eye(15) + dt*F;

% Noise gain matrix
G= [O       O         O  O;
     Cb2n  O         O  O;
     O        -Cb2n O  O;
     O        O         I   O;
     O        O        O   I];
 G = G * dt;
end



