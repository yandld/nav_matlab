clear;
clc;
close all;
format

%% KF结合陀螺仪数据得到硬磁估计
% Adaptive Estimation of Measurement Bias in Three-Dimensional Field
% Sensors with Angular-Rate Sensors: 
Theory and Comparative Experimental Evaluation




dataset = ch_data_import('UranusData.csv');
dt = 0.01;
mag = dataset.imu.mag;
D = mag';
N = length(mag);

X = zeros(6,1);
X(1:3) = mag(:,1);

Q = eye(6)*1^(2);
R = eye(3) * 10^(2);
P = eye(6);

for i=1:N
    gyr = deg2rad(dataset.imu.gyr(:,i));
    y = dataset.imu.mag(:,i);
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
    
end


fprintf("KF得到mag零偏: \n");
b  = X(4:6);
fprintf('硬磁干扰(bias): %.3f %.3f %.3f\n', b);

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
plot(hX(:,1:3));
hold on;
plot(mag', '.');
legend("预测X", "预测Y", "预测Z", "量测X", "量测Y", "量测Z");


fprintf("使用椭球拟合法")
[A, b, expmfs] = magcal(D, 'eye');


fprintf("校准矩阵:\n");
A
fprintf('硬磁干扰(bias): %.3f %.3f %.3f\n', b);

%fprintf( 'away from cetner %.5g\n', norm( b' - c) );
C = (D-b)*A; % calibrated data


figure;
plot3(D(:,1),D(:,2),D(:,3),'LineStyle','none','Marker','X','MarkerSize',2)
hold on
grid(gca,'on')
plot3(C(:,1),C(:,2),C(:,3),'LineStyle','none','Marker', ...
            'o','MarkerSize',2,'MarkerFaceColor','r')
axis equal
xlabel('uT'); ylabel('uT');zlabel('uT')
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off


figure;
subplot(1,2,1);

plot(D(:,1), D(:,3) ,'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('X(uT)'); ylabel('Z(uT)');

hold on;
grid(gca,'on')
plot(C(:,1), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off

subplot(1,2,2);

plot(D(:,2), D(:,3) ,'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('Y(uT)'); ylabel('Z(uT)');
hold on;
grid(gca,'on')
plot(C(:,2), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off


