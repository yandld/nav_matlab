clear;
clc
close all;




%姿态四元数Cb2n b系转到n系
Qb2n = [0.919 0.262  -0.09 -0.282 ]';

% accReading: 静止下加速度计读值, 相当于b系下的静止加速度值
accReading = [0.018 0.531  0.843]';
Cb2n = ch_q2m(Qb2n);
Gn = Cb2n * accReading;
fprintf("example1: 加速度读值转到n系，结果: %.3f %.3f %.3f 可以看到接近于 0 0 1\n", Gn(1), Gn(2), Gn(3));
Gn = ch_qmulv(Qb2n, accReading);
fprintf("example1: 使用四元数旋转函数，可以得到相同结果：  %.3f %.3f %.3f \n", Gn(1), Gn(2), Gn(3));

Gn = [0 0 1]'; %Gn: 静止下，N系下加速度值，应该是重力:
Qn2b = ch_qconj(Qb2n);
Cn2b = ch_q2m(Qn2b);

accB =  Cn2b*Gn;
fprintf("example2: (0 0 1)转到b系,结果: %.3f %.3f %.3f 接近于 accReading: %.3f %.3f %.3f\n", accB(1), accB(2), accB(3), accReading(1), accReading(2), accReading(3));

accB = ch_qmulv(Qn2b, Gn);
fprintf("example1: 使用四元数旋转函数，可以得到相同结果：  %.3f %.3f %.3f \n", accB(1), accB(2), accB(3));

