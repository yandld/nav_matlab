clear;
clc;
close all;

x  = [1 0]';

for i = 0 : pi/6 :2*pi
    theta = i;
    y = ch_rotation_2d(x, theta);
    fprintf("将X %.3f %.3f  逆时针旋转 %.1f° 后得到 %.3f %.3f\n", x(1), x(2), rad2deg(theta), y(1), y(2));
end


