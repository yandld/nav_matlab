clear;
clc;
close all;
format short


% theta = deg2rad(30);
% Cb2nX = ch_rotx(theta);
% fprintf("b系到n系，绕X轴旋转%.3f ° 的旋转矩阵 Cb2nX 为\n", rad2deg(theta));
% Cb2nX
% 
% theta = deg2rad(40);
% Cb2nY = ch_roty(theta);
% fprintf("b系到n系的，绕Y轴旋转%.3f °  的旋转矩阵 Cb2nX为\n", rad2deg(theta));
% Cb2nY
% 
% theta = deg2rad(50);
% Cb2nZ = ch_rotz(theta);
% fprintf("b系到n系的，绕Z轴旋转%.3f °  的旋转矩阵 Cb2nX为\n", rad2deg(theta));
% Cb2nZ


% Ab = [0 0 1]';
% fprintf("b系下有点 A:%.3f %.3f %.3f\n", Ab(1), Ab(2), Ab(3));
% 
% An = Cb2nX*Ab;
% fprintf("b系下点A经过Cb2nX旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));
% 
% An = Cb2nY*Ab;
% fprintf("b系下点A经过Cb2nY旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));
% 
% An = Cb2nZ*Ab;
% fprintf("b系下点A经过Cb2nZ旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));

eul = [30 40 50]';


fprintf("按312顺序(先转Z-然后X-最后Y)旋转，其中X,Y,Z旋转角度为%d° %d° %d° 得到对应的坐标系旋转矩阵：\n", eul(1), eul(2), eul(3));

eul_rad = deg2rad(eul);
Cb2n =  ch_rotz(eul_rad(3)) * ch_rotx(eul_rad(1)) * ch_roty(eul_rad(2));
Cb2n

fprintf("从n系到b系的坐标系旋转矩阵Cn2b:(既Cb2n的转置)\n");
Cn2b = Cb2n';
Cn2b


fprintf("Cn2b也可以通过欧拉角计算:\n");
Cn2b = ch_roty(-eul_rad(2))  * ch_rotx(-eul_rad(1)) * ch_rotz(-eul_rad(3));
Cn2b
fprintf('注意矩阵的旋转顺序 和 欧拉角 符号都要变\n');
 
 

%  [Cnb, Cnbr] = a2mat(eul_rad)











