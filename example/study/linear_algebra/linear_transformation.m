clear;
clc;
close all;

H = [-6 -6 -7 0 7 6 6 -3 -3 0 0 -6; -7 2 1 8 1 2 -7 -7 -2 -2 -7 -7];

[R, ~] = ch_rotation_2d(H, deg2rad(-90));
H = R*H;

x = H(1,:)'; y = H(2,:)';

axis('square');
axis equal
plot(x, y, 'o', x, y, '-');