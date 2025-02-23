clear;
clc;
close all;


%% 参数
N = 100;  % 数据个数
dt = 0.1;  % 积分间隔
omgea = deg2rad(30); %角速度 rad
acc_b = [0 0.47]';  % B 系加速度 m/s^（2） 始终在加速
beta = 0.5; %IMU 里程计互补滤波系数

odom = [
         0.998643790404348
         0.997037958079104
         0.995185707873184
         0.993090753317749
         0.990757316547625
         0.988190128127692
         0.985394426749196
         0.982375958755164
         0.979140977447758
         0.975696242123519
          0.97204901677491
         0.968207068388412
         0.964178664760559
         0.959972571743743
         0.955598049823403
         0.951064849917308
          0.94638320827611
          0.94156384035237
         0.936617933492749
         0.931557138295435
          0.92639355846215
          0.92113973896163
         0.915808652309647
          0.91041368275977
         0.904968608189766
         0.899487579461271
          0.89398509702586
         0.888475984549639
         0.882975359331714
         0.877498599300363
         0.872061306385221
         0.866679266085285
         0.861368403081932
         0.856144732784197
         0.851024308741003
         0.846023165912285
         0.841157259858255
          0.83644240198324
         0.831894191056967
         0.827527941330852
         0.823358607668068
         0.819400708211708
         0.815668245222442
         0.812174624822206
         0.808932576479876
         0.805954073164327
         0.803250253165262
         0.800831344638446
         0.798706593965341
         0.796884199024074
          0.79537124844649
          0.79417366788313
         0.793296174213995
         0.792742238528974
         0.792514058560421
         0.792612541085564
         0.793037294633339
           0.7937866326353
         0.794857586960204
         0.796245931574032
         0.797946215878636
         0.799951807109434
         0.802254941021337
         0.804846779966837
         0.807717477374081
         0.810856247567456
         0.814251439838962
         0.817890615674292
         0.821760628060781
         0.825847701852009
         0.830137514231909
          0.83461527440547
         0.839265801739084
         0.844073601676955
         0.849022938866737
         0.854097907034066
         0.859282495248926
         0.864560650324256
         0.869916335177111
         0.875333583063636
         0.880796547670368
         0.886289549105654
         0.891797115886227
          0.89730402305573
         0.902795326604697
         0.908256394386028
         0.913672933737159
         0.919031016030881
         0.924317098381954
         0.929518042737259
         0.934621132573999
         0.939614087424233
         0.944485075435438
         0.949222724166504
          0.95381612980705
         0.958254864995714
         0.962528985400414
         0.966629035210875
         0.970546051681152
          0.97427156884768];
      
%% 初始值 
p = [0  0]';
v = [1  0]';
yaw = deg2rad(0);


% 生成理想轨迹
for i = 1:N
[p, v, yaw, acc_n] = inernial_navigation_2d(p, v, yaw, acc_b, omgea, dt);
hist.p_gt(:,i) = p;
end

p = [0  0]';
v = [1  0]';
yaw = deg2rad(0);

for i = 1:N
 acc_b = acc_b + randn()*0.05;
[p, v, yaw, acc_n] = inernial_navigation_2d(p, v, yaw, acc_b, omgea, dt);

% 使用里程计计算速度
v_odom(1) = odom(i)*cos(yaw);
v_odom(2) = odom(i)*sin(yaw);
v(1) = v(1) * beta +  v_odom(1) * (1-beta);
v(2) = v(2) * beta +  v_odom(2) * (1-beta);


% 记录数据
hist.acc_n(:,i) = acc_n;
hist.p(:,i) = p;
hist.v(:,i) = v;
hist.yaw(i) = yaw;
hist.vn(i) = norm(v);
end




%% 作图
plot(hist.p(1,:), hist.p(2,:), '.-');
hold on;
plot(hist.p_gt(1,:), hist.p_gt(2,:), '.-');
xlabel("X(m)"); ylabel("Y(m)");
legend("真实轨迹", "仿真轨迹");
title("2D 轨迹");
axis equal

figure;
subplot(3,1,1);
plot(hist.acc_n(1,:));
hold on;
plot(hist.acc_n(2,:));
title("N系下加速度");

subplot(3,1,2);
plot(hist.v(1,:));
hold on;
plot(hist.v(2,:));
title("N系下速度");

subplot(3,1,3);
plot(hist.p(1,:));
hold on;
plot(hist.p(2,:));
title("N系下位置");



function [p, v, yaw, acc_n ] = inernial_navigation_2d(p, v, yaw, acc_b, omgea, dt)

% 更新航向角
yaw = yaw + omgea*dt;

% 计算旋转矩阵
Cnb = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];

% 将B系加速度通过旋转矩阵转到导航系
acc_n = Cnb*acc_b;

% 更新速度
v = v + acc_n.*dt;
% 更新位置
p = p +v.*dt;

end
