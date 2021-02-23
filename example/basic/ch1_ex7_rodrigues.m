clear;
clc;
close all;
format short


rv = [0.6096,  0.5747, 0.3260]';

phi = norm(rv);
u = rv / phi;
fprintf("有等效旋转矢量(RV):\n");
rv

fprintf("其中旋转轴为:(单位矢量)\n");
u
fprintf("rv的大小代表旋转角度:%f deg\n",  rad2deg(phi));


fprintf("使用Rod公式 可以得到结果\n");
R = ch_rv2m(rv);
R

R = expm(ch_askew(rv)); %严龚敏2.1.20
fprintf("使用反对称矩阵幂函数求解可以得到相同结果: \n"); 
R

