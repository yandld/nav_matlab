clear;
clc;
close all;
format short


theta = deg2rad(30);
Cb2nX = ch_rotx(theta);
fprintf("b系到n系，绕X轴旋转%.3f ° 的旋转矩阵 Cb2nX 为\n", rad2deg(theta));
Cb2nX

theta = deg2rad(40);
Cb2nY = ch_roty(theta);
fprintf("b系到n系的，绕Y轴旋转%.3f °  的旋转矩阵 Cb2nX为\n", rad2deg(theta));
Cb2nY

theta = deg2rad(50);
Cb2nZ = ch_rotz(theta);
fprintf("b系到n系的，绕Z轴旋转%.3f °  的旋转矩阵 Cb2nX为\n", rad2deg(theta));
Cb2nZ


Ab = [0 0 1]';
fprintf("b系下有点 A:%.3f %.3f %.3f\n", Ab(1), Ab(2), Ab(3));

An = Cb2nX*Ab;
fprintf("b系下点A经过Cb2nX旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));

An = Cb2nY*Ab;
fprintf("b系下点A经过Cb2nY旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));

An = Cb2nZ*Ab;
fprintf("b系下点A经过Cb2nZ旋转到n系为: %.3f %.3f %.3f\n",An(1), An(2), An(3));
