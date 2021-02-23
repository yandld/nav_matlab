clear;
clc;
close all;
format short


eul_312 = [30, 40, 50]; % [pitch(绕X轴) roll(绕Y轴)  yaw(绕Z轴)]

fprintf("按312顺序(先转Z-然后X-最后Y)旋转，其中X,Y,Z旋转角度为%.3f° %.3f° %.3f° 得到对应的坐标变换矩阵：\n", eul_312(1), eul_312(2), eul_312(3));

eul_312rad = deg2rad(eul_312);
[Cb2n_312, ~] = ch_eul2m(eul_312rad);
Cb2n_312

fprintf("将Cb2n_312转回欧拉角:\n");
[eul_312, ~]  = ch_m2eul(Cb2n_312);
eul_312 = rad2deg(eul_312);

fprintf("坐标变换矩阵转欧拉角:%.3f°(Pitch) %.3f°(Roll) %.3f°(Yaw)\n", eul_312(1), eul_312(2), eul_312(3));


eul_321 = [30, 40, 50]; % [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]

fprintf("\n按321顺序(先转Z-然后X-最后Y)旋转，其中X,Y,Z旋转角度为%.3f° %.3f° %.3f° 得到对应的坐标变换矩阵：\n", eul_321(1), eul_321(2), eul_321(3));

eul_321rad = deg2rad(eul_321);
[~, Cb2n_321] = ch_eul2m(eul_321rad);
Cb2n_321

fprintf("将Cb2n_312转回欧拉角:\n");
[~, eul_321]  = ch_m2eul(Cb2n_321);
eul_321 = rad2deg(eul_321);

fprintf("坐标变换矩阵转欧拉角:%.3f°(Roll) %.3f°(Pitch) %.3f°(Yaw)\n", eul_321(1), eul_321(2), eul_321(3));


