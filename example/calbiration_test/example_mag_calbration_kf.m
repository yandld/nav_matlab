clear;
clc;
close all;
format

%% KF结合陀螺仪数据得到硬磁估计
% Adaptive Estimation of Measurement Bias in Three-Dimensional Field  Sensors with Angular-Rate Sensors:  Theory and Comparative Experimental Evaluation


dataset = ch_data_import('UranusData.csv');

N = length(dataset.imu.gyr);

ratio = 1/5;

cal_len = ratio*N;

cal_gyr = dataset.imu.gyr(:, 1: cal_len);
cal_mag = dataset.imu.mag(:, 1: cal_len);
val_mag = dataset.imu.mag(:, cal_len: end);

dt = 0.01;


X = zeros(6,1);
X(1:3) = cal_mag(:,1);

Q = blkdiag(eye(3)*1^(2), eye(3)*1^(2));
R = eye(3) * 2^(2);
P = eye(6);

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


figure;
subplot(2, 1, 1);
plot(cal_mag');
legend("X", "Y", "Z");
title("磁场测量值");
subplot(2, 1, 2);
plot(cal_gyr');
legend("X", "Y", "Z");
title("陀螺测量值");

figure;
subplot(2, 1, 1);
plot(hX(:,1:3));
hold on;
plot(cal_mag', '.');
legend("预测X", "预测Y", "预测Z", "量测X", "量测Y", "量测Z");
subplot(2, 1, 2);
plot(hX(:,4:6));
title("磁零偏");













fprintf("使用椭球拟合法")
[A, b, expmfs] = magcal(cal_mag', 'eye');


fprintf('硬磁干扰(bias): %.3f %.3f %.3f\n', b);

%fprintf( 'away from cetner %.5g\n', norm( b' - c) );
C = (val_mag'-b)*A; % calibrated data



figure;
plot3(val_mag(:,1),val_mag(:,2),val_mag(:,3),'LineStyle','none','Marker','X','MarkerSize',2)
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

plot(val_mag(1,:), val_mag(3,:),'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('X(uT)'); ylabel('Z(uT)');

hold on;
grid(gca,'on')
plot(C(:,1), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off

subplot(1,2,2);

plot(val_mag(2,:), val_mag(3,:) ,'LineStyle','none','Marker','X','MarkerSize',2);
xlabel('Y(uT)'); ylabel('Z(uT)');
hold on;
grid(gca,'on')
plot(C(:,2), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off

C = (val_mag'-b); 
n = sum(abs(C).^2,2).^(1/2);
std_res = std(n);
fprintf("标准差:%f\n", std_res);

