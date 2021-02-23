clear;
clc;
close all;
format short;


fprintf("\n练习3\n");
Qb2n = [0.836356 -0.32664 0.135299 0.418937]';

fprintf("已知四元数Qb2n:\n");
Qb2n

fprintf('四元数转欧拉角Qb2n\r\n');
ch_q2m(Qb2n)

fprintf("四元数转欧拉角\n");
[~, eul_321] = ch_q2eul(Qb2n);
rad2deg(eul_321)