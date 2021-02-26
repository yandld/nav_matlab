clear;
clc;
format short;


%% 线性代数及其应用第5版 5.8 Iterative Estimates For EienValues

%% EXAMPLE 1
A = [1.8 0.8; 0.2 1.2];
v1 = [4 1]';
x = [-0.5 1]';

for i = 1:8
    x = A*x;
end

x = x / norm(x);
v1= v1 / norm(v1);
% 
% fprintf("可以看到 x 逼近v1的方向\r\n");
% x
% v1

%% EXAMPLE 2
A  = [6 5; 1 2];
x = [0 1]';


for i = 1:8
    x = A*x;
    lanbda = abs(max(x));
    x  = x / lanbda;

end

x
lanbda

