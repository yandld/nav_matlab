clear;
clc;
close all;
format short


V= rand(3,1);
nV = norm(V);

fprintf("生成空间中任意一点:%.3f %.3f %.3f\n", V(1), V(2), V(3));


V2 = eye(3) + (sin(nV) / nV )* ch_askew(V)  + ((1-cos(nV)) / nV^(2)) *ch_askew(V)^(2); %严龚敏2.2.18
fprintf("使用Rod旋转公式得到旋转后向量: %.3f %.3f %.3f\n", V2(1), V2(2), V2(3)); 
V2 = expm(ch_askew(V)); %严龚敏2.1.20
fprintf("使用反对称矩阵幂函数求解可以得到相同结果: %.3f %.3f %.3f\n", V2(1), V2(2), V2(3)); 


