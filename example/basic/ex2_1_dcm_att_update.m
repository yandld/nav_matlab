clear;
clc
close all;

%% 已知
w = [2 -3 5]';
dt = 0.01; %姿态更新周期: 0.01s = 100Hz
dur = 100; %积分时长 单位s


N = dur / dt; %积分次数
eul = zeros(N, 3);
fprintf("已知b系角速度(陀螺仪输出为):%.3fdeg/s %.3fdeg/s %.3fdeg/s\n", w(1), w(2), w(3));


Cb2n = eye(3);
theta = deg2rad(w)*dt; %定轴转动下: 等效旋转矢量 = 角增量 = 角速度*dt


for i = 1:N
    C_m2m_1 = ch_rv2m(theta);
    Cb2n = Cb2n * C_m2m_1;

    
    %单位阵正交化
    Cb2n = ch_mnormlz(Cb2n);

    %记录每一步的欧拉角
    eul(i,:) = rad2deg(ch_m2eul(Cb2n));
end


plot(eul)
legend("PITCH(deg)", "ROLL(deg)", "YAW(deg)");

tmp =eul(end,:);
fprintf("最终欧拉角: pich:%.4f° roll:%.4f° yaw:%.4f°\n", tmp(1), tmp(2), tmp(3));

