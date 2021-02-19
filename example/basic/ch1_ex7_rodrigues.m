clear;
clc;
close all;
format short


rv = [0.6096,  0.5747, 0.3260]';

phi = norm(rv);
u = rv / phi;
fprintf("有旋转矢量(rv):\n");
rv

fprintf("其中旋转轴为:\n");
u
fprintf("rv的大小代表旋转角度:%f rad",  phi);


fprintf("旋转矢量:%.3f %.3f %.3f\n", rv(1), rv(2), rv(3));


R = eye(3) + (sin(phi) / phi )* ch_askew(rv)  + ((1-cos(phi)) / phi^(2)) *ch_askew(rv)^(2); %严龚敏2.2.18
fprintf("使用Rod旋转公式得到旋转矩阵:\n");
R

R = expm(ch_askew(rv)); %严龚敏2.1.20
fprintf("使用反对称矩阵幂函数求解可以得到相同结果: \n"); 
R

fprintf("使用库函数 rv2m 可以得到相同结果:(实际还是用的Rod旋转公式)\n");
R = ch_rv2m(rv);
R


