clear;
clc;
close all;


%参考真值:
GT = [ 13780293.30, -20230949.12, 10441947.44];


%星历参数
sqrtA = 5153.65531;
toe = 244800;
deln = 4.249105564E-9;
M0 = -1.064739758;
e = 0.005912038265;
omg = -1.717457876;
i0 = 0.9848407943;
idot = 7.422851197E-51;
OMG0 = 1.038062244;
OMGd = -8.151768125E-9;
cus = 2.237036824E-6;
cuc = 3.054738045E-7;
crs = 2.53125;
crc = 350.53125;
cis = 8.940696716E-8;
cic = -8.381903172E-8;

%  当前GPS时间
t = 239050.7223;

toc = 0;
a0 = 0;
a1 = 0;
a2 = 0;

[XYZ, sv_dt] = ch_sat_pos(t, toc, a0, a1, a2, crs, deln, M0, cuc, e, cus, sqrtA, toe, cic, OMG0, cis, i0, crc, omg, OMGd, idot);

fprintf("卫星位置: %f %f %f\r\n", XYZ(1), XYZ(2), XYZ(3));
fprintf("与真实值相差：%fm\r\n", norm(GT -XYZ'));



