clc
clear
close all


%% 说明
% UWB IMU 融合算法，采用误差卡尔曼15维经典模型，伪距组合
% PR(TOF) 伪距：        UWB硬件给出的原始测量距离值
% IMU:                       加速度(3) 3轴陀螺(3) 共6维
% noimal_state:           导航方程状态: 位置(3) 速度(3) 四元数(4) 共10维
% err_state:                 KF误差状态: 位置误差(3) 速度误差(3) 失准角(3) 加速度计零偏(3) 陀螺零偏(3) 共15维
% du:                          零偏反馈: 加速度计零偏(3) 陀螺零偏(3)

%% 读取数据集
load datas2;
dataset = datas;


N = length(dataset.imu.time);
dt = mean(diff(dataset.imu.time));

% 故意删除一些基站及数据，看看算法在基站数量很少的时候能否有什么奇迹。。 
% dataset.uwb.anchor(:,1) = [];
% dataset.uwb.tof(1,:) = [];


% EKF融合使用的基站个数，融合算法最少2个基站就可以2D定位
%dataset.uwb.cnt = size(dataset.uwb.anchor, 2);


m_div_cntr = 0;                         % 量测分频器
m_div = 50;                                 % 每m_div次量测，才更新一次EKF量测(UWB更新),  可以节约计算量 或者 做实验看效果
UWB_LS_MODE = 2;                 % 2 纯UWB解算采用2D模式， 3：纯UWB解算采用3D模式
UWB_EKF_UPDATE_MODE = 2; % EKF 融合采用2D模式，   3: EKF融合采用3D模式

%% 数据初始化
out_data.uwb = [];
out_data.uwb.time = dataset.uwb.time;
out_data.imu.time = dataset.imu.time;
out_data.uwb.anchor = dataset.uwb.anchor;
pr = 0;
last_pr = 0;

%% 滤波参数初始化
settings = uwb_imu_example_settings();
R = diag(ones(dataset.uwb.cnt, 1)*settings.sigma_uwb^(2));
noimal_state = init_navigation_state(settings);
err_state = zeros(15, 1);

%使用第一帧伪距作为初始状态
pr = dataset.uwb.tof(:, 1);
noimal_state(1:3) = ch_multilateration(dataset.uwb.anchor, [ 0 0 0]',  pr', 3);

du = zeros(6, 1);
[P, Q1, Q2, ~, ~] = init_filter(settings);

fprintf("共%d帧数据, IMU采样频率:%d Hz 共运行时间 %d s\n", N,  1 / dt, N * dt);
fprintf("UWB基站个数:%d\n", dataset.uwb.cnt);
fprintf("UWB量测更新频率为:%d Hz\n", (1 / dt) / m_div);
fprintf("UWB EKF量测更新模式: %dD模式\n", UWB_EKF_UPDATE_MODE);
fprintf("纯UWB最小二乘解算: %dD模式\n", UWB_LS_MODE);
fprintf("EKF 滤波参数:\n");
settings
fprintf("开始滤波...\n");


for k=1:N
    
    acc = dataset.imu.acc(:,k);
    gyr = dataset.imu.gyr(:,k);
    
    % 反馈
    acc = acc + du(1:3);
    gyr = gyr + du(4:6);
    
    % 捷联惯导
    p = noimal_state(1:3);
    v = noimal_state(4:6);
    q =  noimal_state(7:10);
    
    [p, v, q] = ch_nav_equ_local_tan(p, v, q, acc, gyr, dt, [0, 0, -9.8]'); % 东北天坐标系，重力为-9.8
    
    %   小车假设：基本做平面运动，N系下Z轴速度基本为0，直接给0
     v(3) = 0;
    
    noimal_state(1:3) = p;
    noimal_state(4:6) = v;
    noimal_state(7:10) = q;
    out_data.eul(k,:) = ch_q2eul(q);
    
    % 生成F阵   G阵
    [F, G] = state_space_model(noimal_state, acc, dt);
    
    %卡尔曼P阵预测公式
    P = F*P*F' + G*blkdiag(Q1, Q2)*G';
    
    %记录数据
    out_data.x(k,:)  = noimal_state;
    out_data.delta_u(k,:) = du';
    out_data.diag_P(k,:) = trace(P);
    
    
    %% EKF UWB量测更新
    m_div_cntr = m_div_cntr+1;
    if m_div_cntr == m_div
        m_div_cntr = 0;
        
        pr = dataset.uwb.tof(1:dataset.uwb.cnt, k);
        
        %判断两次PR 差，如果差太大，则认为这个基站PR比较烂，不要了。相当于GNSS里的挑星
        %                         arr = find(abs(pr - last_pr) < 0.05);
        %                         last_pr = pr;
        %                         out_data.good_anchor_cnt(k,:) = length(arr); %记录挑出来的基站数
        %
        %                         if(isempty(arr))
        %                             continue;
        %                         end
        %
        %                         %构建 剔除不好的基站之后的基站位置矩阵和R矩阵
        %                         pr = pr(arr);
        %                         anch = dataset.uwb.anchor(:, arr);
        %                         R1 = R(arr, arr);
        
        % 算了不挑基站了，所有基站直接参与定位，其实也差不太多
        anch = dataset.uwb.anchor;
        R1 = R;
        
        %量测方程
        [Y, H]  = uwb_hx(noimal_state, anch, UWB_EKF_UPDATE_MODE);
        
        % 卡尔曼公式，计算K
        S = H*P*H'+R1; % system uncertainty
        residual = pr - Y; %residual 或者叫信息
        
        %% 根据量测置信度给R一些增益   Self-Calibrating Multi-Sensor Fusion with Probabilistic
        %Measurement Validation for Seamless Sensor Switching on a UAV, 计算量测可靠性
        %
        L = (residual'*S^(-1)*residual);
        out_data.L(k,:) = L;
        
        %         if(L > 20 ) %如果量测置信度比较大，则更不相信量测
        %             S = H*P*H'+R1*5;
        %         end
        
        K = (P*H')/(S);
        err_state = [zeros(9,1); du] + K*(residual);
        
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
    
    
        %% 车载约束：Z轴速度约束： B系下 Z轴速度必须为0(不能钻地).. 可以有效防止Z轴位置跳动 参考https://kth.instructure.com/files/677996/download?download_frd=1 和 https://academic.csuohio.edu/simond/pubs/IETKalman.pdf
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
    
    
end

fprintf("开始纯UWB最小二乘位置解算...\n");
%% 纯 UWB 位置解算
j = 1;
uwb_pos = [0 0 0]';
N = length(dataset.uwb.time);

for i=1:N
    pr = dataset.uwb.tof(:, i);
    % 去除NaN点
    %if all(~isnan(pr)) == true
        
        uwb_pos = ch_multilateration(dataset.uwb.anchor, uwb_pos,  pr', UWB_LS_MODE);
        out_data.uwb.pos(:,j) = uwb_pos;
        j = j+1;
    %end
end
fprintf("计算完成...\n");

%% plot 数据
out_data.uwb.tof = dataset.uwb.tof;
out_data.uwb.fusion_pos = out_data.x(:,1:3)';
demo_plot(dataset, out_data);



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
% anchor: 基站坐标  M x N: M:3(三维坐标)，  N:基站个数
% dim:  2: 二维  3：三维
function [Y, H] = uwb_hx(x, anchor, dim)
N = size(anchor,2); %基站个数

pos = x(1:3);
if(dim)== 2
    pos = pos(1:2);
    anchor = anchor(1:2, 1:N);
    %  uwb.anchor
end

Y = [];
H = [];
residual = repmat(pos,1,N) - anchor(:,1:N);
for i=1:N
    
    if(dim)== 2
        H = [H ;residual(:,i)'/norm(residual(:,i)),zeros(1,13)];
    else
        H = [H ;residual(:,i)'/norm(residual(:,i)),zeros(1,12)];
    end
    Y = [Y ;norm(residual(:,i))];
end



end
