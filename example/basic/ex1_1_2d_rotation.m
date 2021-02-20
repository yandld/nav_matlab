clear;
clc;
close all;

X = [1, 0]';

for i = 0 : pi/4 :2*pi
    theta = i;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    Y = R*X;
    fprintf("将X [%.3f %.3f]  逆时针旋转 %.1f° 后得到 %.3f %.3f\n", X(1), X(2), rad2deg(theta), Y(1), Y(2));
end


