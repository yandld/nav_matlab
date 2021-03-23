clear;
clc
close all;
format 

%% 一个简单的失准角例子

% M_gen = ch_eul2m(deg2rad([30,40,50.2]))

Cb2n = [
    0.2462   -0.6634    0.7066
    0.7934    0.5567    0.2462
   -0.5567    0.5000    0.6634
];

Cb2np = [
    0.2434   -0.6654    0.7057
    0.7943    0.5544    0.2487
   -0.5567    0.5000    0.6634
    ];

fprintf("Cb2n: 真实姿态阵):\n");
fprintf("Cb2np: 计算导航系姿态阵(有误差的)):\n");


fprintf("失准角反对称阵为:\n");
phi_skew = -(Cb2np * Cb2n'  - eye(3));
phi_skew


phi = [-phi_skew(2,3), phi_skew(1,3), phi_skew(1,2)];
phi = rad2deg(phi);
fprintf("失准角:%f %f %f (deg)\n", phi(1), phi(2), phi(3));
