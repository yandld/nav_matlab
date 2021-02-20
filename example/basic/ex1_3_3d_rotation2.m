clear;
clc;
close all;
format short


eul = [30 40 50]';


fprintf("按312顺序(先转Z-然后X-最后Y)旋转，其中X,Y,Z旋转角度为%d° %d° %d° 得到对应的坐标系旋转矩阵：\n", eul(1), eul(2), eul(3));

eul_rad = deg2rad(eul);
Cb2n_312 =  ch_rotz(eul_rad(3)) * ch_rotx(eul_rad(1)) * ch_roty(eul_rad(2));
Cb2n_312


fprintf("按321顺序(先转Z-然后Y-最后Z)旋转，其中X,Y,Z旋转角度为%d° %d° %d° 得到对应的坐标系旋转矩阵：\n", eul(1), eul(2), eul(3));

Cb2n_321 =  ch_rotz(eul_rad(3)) * ch_roty(eul_rad(2)) * ch_rotx(eul_rad(1));
Cb2n_321

fprintf("从n系到b系的坐标系旋转矩阵Cn2b:(既Cb2n的转置)\n");
Cn2b_312 = Cb2n_312';
Cn2b_312


fprintf("Cn2b也可以通过欧拉角计算:， 注意矩阵的旋转顺序 和 欧拉角 符号都要变\n");
Cn2b_312 = ch_roty(-eul_rad(2))  * ch_rotx(-eul_rad(1)) * ch_rotz(-eul_rad(3));
Cn2b_312


% 验证
%   [Cb2n_312 Cb2n_321] = ch_eul2m(eul_rad);
% Cb2n_312
% Cb2n_321
% 








