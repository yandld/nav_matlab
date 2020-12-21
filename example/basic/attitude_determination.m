clear;
clc;
close all;
%% 单矢量定姿

accReading = [0.168 0.322 0.963]';
Cb2n = ch_sv2atti(accReading);

eul = rad2deg( ch_m2eul(Cb2n));
Qb2n = ch_m2q(Cb2n);

fprintf("单矢量定姿: Roll:%.2f° Pitch:%.2f° Yaw:%.2f°\n", eul(1), eul(2), eul(3));
fprintf("单矢量定姿: 四元数: %.3f %.3f %.3f %.3f\n", Qb2n(1), Qb2n(2), Qb2n(3), Qb2n(4));


%% 加速度地磁双矢量定姿
clear;

accReading = [0.203 0.369 0.902]';
magReading  = [5.6650 -26.7632 -41.3791]'; %注意地磁必须是校正过的

vb1 = accReading / norm(accReading);
vb2 = magReading / norm(magReading);

vn1 = [0 0 1]';

%求VN2
Cb2n = ch_sv2atti(accReading);
vtmp = Cb2n*vb2;

% NWU系
vn2(1) = sqrt(vtmp(1)^(2) + vtmp(2)^(2));
vn2(2) = 0;
vn2(3) = vtmp(3);
vn2 = vn2';

%定姿
Cb2n = ch_dv2atti(vn1, vn2, vb1, vb2);

%输出结果
eul = rad2deg(ch_m2eul(Cb2n));
Qb2n = ch_m2q(Cb2n);

fprintf("双矢量定姿: Roll:%.2f° Pitch:%.2f° Yaw:%.2f°\n", eul(1), eul(2), eul(3));
fprintf("双矢量定姿: 四元数: %.3f %.3f %.3f %.3f\n", Qb2n(1), Qb2n(2), Qb2n(3), Qb2n(4));


