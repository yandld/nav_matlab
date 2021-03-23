clc;
clear;
close all;


R = 0.05; %量测噪声
Q = 0.1*eye(2); %过程噪声
u = -2; %控制输入， -2m/s^(2)
dt = 0.5;
X = [0 5]'; %初始状态
P = [0.01 0; 0 1]; %初始系统方差
H = [1 0];
y = 2.2; %第一次量测信息

F =  [1 dt; 0 1]; %过程传递矩阵

%  predict
X =F*X + [0 dt]'*u;
P = F*P*F' + Q;
fprintf("进行一步预测后状态:\n");
X
fprintf("进行一步预测后状态方差:\n");
P


% update
K = P*H'*(H*P*H' +R )^(-1);
X = X + K*(y - H*X);
P = (eye(2) - K*H)*P;

fprintf("量测更新后系统状态\n");
X

fprintf("量测更新后系统方差(不确定性)变小了:\n");
P









