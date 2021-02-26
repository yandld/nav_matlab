%% Start of script
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal



%% madicgick 
% https://blog.csdn.net/nemol1990/article/details/23102643
% ax ay az mx my mz 是传感器读数
syms q0 q1 q2 q3 ax ay az  mx my mz bx by bz real 

%% acc  q 是自变量  x y z 是目标值

x = 2*(q1*q3 - q0*q2) -ax;
y = 2*(q2*q3 +q0*q1) - ay;
z = 1 - 2*q1^2 - 2*q2^2 -az;

Jacc = jacobian([x;y;z],[q0 q1 q2 q3]);
Jacc
Jacc;
cost_acc =  Jacc' * [x;y;z];
cost_acc = collect(cost_acc, [q0 q1 q2 q3])

%% mag
x = 2*bx*(0.5 - q2^2 - q3^2) + 2*bz*(q1*q3 -  q0*q2) - mx;
y = 2*bx*(q1*q2 - q0*q3)       + 2*bz*(q0*q1 + q2*q3) - my;
z = 2*bx*(q0*q2 +q1*q3)       + 2*bz*(0.5 - q1^2 - q2^2) - mz;
Jmag = jacobian([x; y; z],[q0 q1 q2 q3]);
cost_mag = Jmag'*[x;y;z];
collect(cost_mag)


