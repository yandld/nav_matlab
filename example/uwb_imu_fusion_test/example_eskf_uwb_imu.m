clc
clear
close all


%% Motion Process, Measurement model and it's derivative
h_func = @uwb_h;
dh_dx_func = @err_uwb_h;

%N = size(u,2);
imu_iter = 1;
uwb_iter = 1;
ISPUTCHAR = 0;

%% load data set
load dataset2;

dt = dataset.imu.time(2) - dataset.imu.time(1);

section = [5000  7000]*4;

u = [dataset.imu.acc; dataset.imu.gyr];

u = u(:,section(1) :section(2));

dataset.imu.time = dataset.imu.time(1,section(1) :section(2));
dataset.uwb.time =  dataset.uwb.time(1,section(1)/4 :section(2)/4);
dataset.uwb.tof = dataset.uwb.tof(:,section(1)/4 : section(2)/4);

dataset.uwb.anchor = [dataset.uwb.anchor(:,1:5); [0.01 0 0 0 0]];
dataset.uwb.cnt = 5;
N = length(u);

MeasureNoiseVariance = [2.98e-03, 2.9e-03,1.8e-03, 1.2e-03, 2.4e-03];  %%%%  UWB Ranging noise

R = diag(MeasureNoiseVariance(1:dataset.uwb.cnt));
p_div = 0;

%% 打印原始数据
ch_plot_imu('time', 1:length(dataset.imu.acc), 'acc', dataset.imu.acc' / 9.8, 'gyr', rad2deg(dataset.imu.gyr'));

%% out data init
out_data.uwb = [];
out_data.uwb.time = dataset.uwb.time;
out_data.imu.time = dataset.imu.time;
out_data.uwb.anchor = dataset.uwb.anchor;

%% load settings
settings = gnss_imu_local_tan_example_settings();
noimal_state = init_navigation_state(settings);
delta_u_h = zeros(6, 1);
[P, Q1, Q2, ~, ~] = init_filter(settings);


for k=1:N
   
    u_h = u(:,k);
    
     % 反馈
    u_h = u_h + delta_u_h;
    
    % 捷联惯导
	[noimal_state(1:3), noimal_state(4:6), noimal_state(7:10)] = ch_nav_equ_local_tan(noimal_state(1:3), noimal_state(4:6), noimal_state(7:10), u_h(1:3), u_h(4:6), dt, [0, 0, 9.7803698]');
        
    p_div = p_div+1;
    if p_div == 1
        %Get state space model matrices
        [F, G] = state_space_model(noimal_state, u_h, dt*p_div);
        
        %Time update of the Kalman filter state covariance.
        P = F*P*F' + G*blkdiag(Q1, Q2)*G';
        p_div = 0;
    end
    
    if ISPUTCHAR == 1
        cprintf('text', 'time: %8.3f s, Position = [%0.2f %0.2f %0.2f] m, Velocity = [%0.3f %0.3f %0.3f] m/s Position Variance = [%0.5f %0.5f %0.5f], Velocity Variance = [%0.5f %0.5f %0.5f]m/s^2\n',...
            dataset.imu.time(imu_iter) ,X(1),X(2),X(3),X(4),X(5),X(6), P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6));
    end
    
    
    %% EKF UWB量测更新
    if uwb_iter < length(dataset.uwb.time)  && abs(dataset.imu.time(imu_iter) - dataset.uwb.time(uwb_iter))  < 0.01
        
        y = dataset.uwb.tof(:,uwb_iter);
        y = y(1:dataset.uwb.cnt);
        
        % bypass Nan
        if sum(isnan(y)) == 0
            [~,H] = dh_dx_func(noimal_state, dataset.uwb);
            
            % Calculate the Kalman filter gain.
            K=(P*H')/(H*P*H'+R);
            
            % NLOS elimation
            t = h_func(noimal_state, dataset.uwb);
            if uwb_iter > 50
                for i = 1:length(y)
                    if abs(y(i) - t(i))  > 0.9
                        y(i) = t(i); %丢弃这次量测，直接认为这次量测就是预测误差
                    end
                end
            end
            
            err_state = [zeros(9,1); delta_u_h] + K*(y - h_func(noimal_state, dataset.uwb));
            
            % 反馈速度位置
            noimal_state(1:6) = noimal_state(1:6) + err_state(1:6);
            
            % 反馈姿态
            q = noimal_state(7:10);
            q = ch_qmul(ch_qconj(q), ch_rv2q(err_state(7:9)));
            q = ch_qconj(q);
            noimal_state(7:10) = q;
            
            delta_u_h = err_state(10:15);
            
            % Update the Kalman filter state covariance.
            P=(eye(15)-K*H)*P;
            
            if ISPUTCHAR == 1
                cprintf('err', 'time: %8.3f s, Position [%0.2f %0.2f %0.2f] m, Velocity [%0.3f %0.3f %0.3f] m/s Position Variance = [%0.5f %0.5f %0.5f], Velocity Variance = [%0.5f %0.5f %0.5f]m/s^2\n\n\n',...
                    dataset.uwb.time(uwb_iter) ,X(1),X(2),X(3),X(4),X(5),X(6), P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6));
            end
        end
        uwb_iter = uwb_iter + 1;
    end
    
    out_data.x(k,:)  = noimal_state;
    out_data.delta_u(k,:) = delta_u_h';
    out_data.diag_P(k,:) = trace(P);
    imu_iter = imu_iter + 1;
end

%% 纯 UWB 位置解算
cnt = 1;
uwbxyz = [1 2 3]';

for uwb_iter=1:length(dataset.uwb.time)
    y = dataset.uwb.tof(:, uwb_iter);
    % 去除NaN点
    if all(~isnan(y)) == true
        uwbxyz = ch_multilateration(dataset.uwb.anchor, uwbxyz,  y');
        out_data.uwb.pos(:,cnt) = uwbxyz;
         cnt = cnt+1;
    end

end


%% show all data
out_data.uwb.tof = dataset.uwb.tof;
out_data.uwb.fusion_pos = out_data.x(:,1:3)';

fusion_display(out_data, []);

%%  Init navigation state
function x = init_navigation_state(settings)

roll = 0;
pitch = 0;

% Initial coordinate rotation matrix
q = ch_eul2q([roll pitch settings.init_heading]);
x = [zeros(6,1); q];
end


%%  Init filter
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

%%  State transition matrix
function [F,G] = state_space_model(x, u, t)
Cb2n = ch_q2m(x(7:10));

% Transform measured force to force in the tangent plane coordinate system.
sf = Cb2n * u(1:3);
St = ch_askew(sf);

% Only the standard errors included
O = zeros(3);
I = eye(3);
F = [ O I   O O       O;
    O O St Cb2n O;
    O O O O       -Cb2n;
    O O O O       O;
    O O O O       O];

% Approximation of the discret  time state transition matrix
F = eye(15) + t*F;

% Noise gain matrix
G=t*[O       O         O  O;
    Cb2n  O         O  O;
    O        -Cb2n O  O;
    O        O         I   O;
    O        O        O   I];
end


