clear;
clc
close all;

%%
% Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2nd Edition, by Paul D. Groves
% EXAMPLE 5.1
% Inertial Navigation in Two Dimensions

%% INTPUS 
p = [0; 0];
v = [0;  0];
y = deg2rad(45);


w = [0.5; 0.2; 0.1; -0.1];              % 角速度输入
dt = 0.5;                                    % 积分间隔
a = [2  0.1; 5 0.1; 2 -0.05; 0 0];  % 加速度输入

for i = 1:4
y_ = y;               % 上一次的航向角
y = y + w(i)*dt;  % 更新航向角

% transform acceration to p frame, get a average value
Cnb_ = [cos(y_) -sin(y_); sin(y_) cos(y_)];
Cnb = [cos(y) -sin(y); sin(y) cos(y)];
Cnb = (Cnb_ + Cnb) / 2;

a_n = Cnb*a(i,:)'; %N系下加速度

%更新速度
v_ = v; 
v = v + a_n*dt;

%更新位置
p = p + (v_ +v)*0.5*dt;

end

fprintf("最终位置:(m)\n");
p
