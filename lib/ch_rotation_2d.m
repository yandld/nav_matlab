function y = ch_rotation_2d(x, theta)
% ch_rotation_2d: 
%
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
y = R*x;