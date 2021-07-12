clear;
clc;
close all;


%参考真值:
GT = [ 13780293.30, -20230949.12, 10441947.44];


%星历参数
sqrtA = 5153.65531;
toe = 244800;
Delta_n = 4.249105564E-9;
M0 = -1.064739758;
e = 0.005912038265;
omega = -1.717457876;
i0 = 0.9848407943;
iDOT = 7.422851197E-51;
OMEGA = 1.038062244;
OMEGA_DOT = -8.151768125E-9;
Cus = 2.237036824E-6;
Cuc = 3.054738045E-7;
Crs = 2.53125;
Crc = 350.53125;
Cis = 8.940696716E-8;
Cic = -8.381903172E-8;

%  当前GPS时间
t = 239050.7223;

toc = 0;
a0 = 0;
a1 = 0;
a2 = 0;

[X, Y, Z, sv_dt] = ch_sat_pos(t, toc, a0, a1, a2, Crs, Delta_n, M0, Cuc, e, Cus, sqrtA, toe, Cic, OMEGA, Cis, i0, Crc, omega, OMEGA_DOT, iDOT);

fprintf("卫星位置: %f %f %f\r\n", X, Y, Z);
fprintf("与真实值相差：%fm\r\n", norm(GT - [X Y Z]));



