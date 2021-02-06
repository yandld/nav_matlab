clear;
clc
close all;

%% 已知
Qb2n = [0.936 -0.259 -0.233 -0.046]';

eul = ch_q2eul(Qb2n, '312');
eul = rad2deg(eul);
fprintf("Qb2n对应的欧拉角 %.3f° %.3f° %.3f°\n", eul(1), eul(2), eul(3));

Cb2n = ch_q2m(Qb2n);
eul = ch_m2eul(Cb2n, '312'); 
eul = rad2deg(eul);
fprintf("Cb2n对应的欧拉角 %.3f° %.3f° %.3f°\n", eul(1), eul(2), eul(3));

fprintf("两种方法应该得到相同的结果\n");
