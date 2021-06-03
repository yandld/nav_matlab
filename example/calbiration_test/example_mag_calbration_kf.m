clear;
clc;
close all;
format

%% KF结合陀螺仪数据得到硬磁估计
% Adaptive Estimation of Measurement Bias in Three-Dimensional Field  Sensors with Angular-Rate Sensors:  Theory and Comparative Experimental Evaluation


dataset = ch_data_import('UranusData.csv');

N = length(dataset.imu.gyr);

% 不使用全部数据集，只使用一部分
ratio = 1/1.1;

cal_len = ratio*N;

cal_gyr = dataset.imu.gyr(:, 1: cal_len);
cal_mag = dataset.imu.mag(:, 1: cal_len);
val_mag = dataset.imu.mag(:, cal_len: end);

dt = 0.02; % 采样dt

X = zeros(6,1);
X(1:3) = cal_mag(:,1);

Q = blkdiag(eye(3)*0.1^(2), eye(3)*0.1^(2));
R = eye(3) * 1^(2);
P = eye(6)*100;

for i=1:cal_len
    gyr = deg2rad(cal_gyr(:,i));
    y = cal_mag(:,i);
    sk = ch_askew(gyr);
    F = [-sk, sk; zeros(3,6)];
    F = eye(6) + F*dt;
    
    % predict
    X = F*X;
    P = F*P*F' + Q;
    
    % update
    H = [eye(3) zeros(3,3)];
    
    K = P*H'*(H*P*H' +R )^(-1);
    X = X + K*(y - H*X);
    P = (eye(6) - K*H)*P;
    
    hX(i,:) = X;
    hP(i,:) = trace(P);
end


fprintf("KF得到mag零偏: \n");
b  = X(4:6);
fprintf('硬磁干扰(bias): %.3f %.3f %.3f\n', b);


C = (val_mag'-b'); 
n = sum(abs(C).^2,2).^(1/2);
std_res = std(n);
fprintf("标准差:%f\n", std_res);

% 
% figure;
% subplot(2, 1, 1);
% plot(cal_mag');
% legend("X", "Y", "Z");
% title("磁场测量值");
% subplot(2, 1, 2);
% plot(cal_gyr');
% legend("X", "Y", "Z");
% title("陀螺测量值");

figure('Name', 'KF估计硬磁');
subplot(2, 1, 1);
title("KF状态量");
plot(hX(:,1:3));
hold on;
plot(cal_mag', '.');
legend("预测X", "预测Y", "预测Z", "量测X", "量测Y", "量测Z");
subplot(2, 1, 2);
plot(hX(:,4:6));
title("磁零偏");


%% https://github.com/pronenewbits/Arduino_AHRS_System

fprintf("RLS估计硬磁:\n");

X = zeros(4,1);
P = eye(4)*10;
lamda = 1.0;

for i=1:cal_len
    y = cal_mag(:,i);
    H = [y; 1]';
    K = P*H' / (H*P*H' + lamda);
    P = (eye(4) - K*H)*P / lamda;
    X = X + K*(y'*y - H*X);

    hP(i,:) = trace(P);
    htheta(i,:) = X;
end

fprintf("硬磁干扰(bias): %.3f %.3f %.3f, 磁场强度: %.3f\n", X(1)/2, X(2)/2, X(3)/2, sqrt(X(4)));

figure('Name', 'RLS估计硬磁');
subplot(2, 1, 1);
plot(hP);
title("P");
subplot(2, 1, 2);

htheta(:,1:3) = htheta(:,1:3) /2;
htheta(:,4)  = real(sqrt(htheta(:,4)));

plot(htheta);
title("theta");
legend("X", "Y", "Z", "磁场强度");





fprintf("使用椭球拟合法")
[A, b, expmfs] = magcal(cal_mag', 'auto');
A
fprintf('硬磁干扰(bias): %.3f %.3f %.3f\n', b);

C = (val_mag'-b); 
n = sum(abs(C).^2,2).^(1/2);
std_res = std(n);
fprintf("标准差:%f\n", std_res);



figure;
plot3(cal_mag(1,:),cal_mag(2,:),cal_mag(3,:),'LineStyle','none','Marker','X','MarkerSize',2)
hold on
plot3(C(:,1),C(:,2),C(:,3),'LineStyle','none','Marker',  'o','MarkerSize',2,'MarkerFaceColor','r')
axis equal
xlabel('uT'); ylabel('uT');zlabel('uT')
legend('未校准s', '已校准');
axis equal

figure;
subplot(1,2,1);

plot(val_mag(1,:), val_mag(3,:),'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('X(uT)'); ylabel('Z(uT)');
hold on;
plot(C(:,1), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('未校准s', '已校准');
axis equal

subplot(1,2,2);
plot(val_mag(2,:), val_mag(3,:) ,'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('Y(uT)'); ylabel('Z(uT)');
hold on;
plot(C(:,2), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('未校准s', '已校准');
axis equal
hold off

