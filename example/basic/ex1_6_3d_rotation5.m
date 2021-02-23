clear;
clc
close all;

%% 已知
Vn = [0 0 1];
Qa2n = [0.981 0.010 -0.191 0.011]';
Qb2n = [0.936 -0.259 -0.233 -0.046]';

Va = [0.375 0.021 0.876]';
Vb = [0.465  -0.447 0.712]';


%% 四元数转成矩阵
Ca2n = ch_q2m(Qa2n);
Cb2n = ch_q2m(Qb2n);

%%计算 Ca2b, Qa2b
Ca2b = Cb2n' * Ca2n;
Qa2b = ch_m2q(Ca2b);

%% 转换并显示结果
fprintf("b2c的四元数为:%.3f %.3f %.3f %.3f\n", Qa2b(1), Qa2b(2), Qa2b(3), Qa2b(4));
Vb_ = Ca2b*Va;
fprintf("通过Ca2b 将Va转成Vb: %.3f %.3f %.3f, 约等于: %.3f %.3f %.3f \n", Vb_(1), Vb_(2), Vb_(3), Vb(1), Vb(2), Vb(3));
 
Qa2b = ch_qmul(ch_qconj(Qb2n), Qa2n);
Vb_ = ch_qmulv(Qa2b, Va);
 fprintf("通过Qa2b 将Va转成Vb: %.3f %.3f %.3f, 约等于: %.3f %.3f %.3f \n", Vb_(1), Vb_(2), Vb_(3), Vb(1), Vb(2), Vb(3));
 
 Qb2a = ch_qconj(Qa2b);
 Va_ = ch_qmulv(Qb2a, Vb);
 fprintf("通过Qb2a 将Vb转成Va: %.3f %.3f %.3f, 约等于: %.3f %.3f %.3f \n", Va_(1), Va_(2), Va_(3), Va(1), Va(2), Va(3));

