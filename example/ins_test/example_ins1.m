%% INS惯导解算
clear;
clc;
close all;

%%
Fs = 100;
N = Fs*10;

gyr = [0.1, 0.2, 0.3];
acc = [0, 0, 1];

acc = acc * 9.795;
gyr = deg2rad(gyr);

% 捷联惯导解算
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';

for i=1:N
    [p ,v, q] = ch_nav_equ_local_tan(p, v, q, acc', gyr' , 1 / Fs, [0, 0, -9.795]');
    pos(i,:) = p;
    eul(i,:) = rad2deg(ch_q2eul(q));
end


plot(pos);
title("位置解算结果");
legend("X", "Y", "Z");

figure;
plot(eul);
title("欧拉角");
legend("P", "R", "Y");

fprintf('纯积分测试: 陀螺bias(rad):%.3f %.3f %.3f\n', gyr(1), gyr(2), gyr(3));
fprintf('纯积分测试: 加计bias(m/s^(2)):%.3f %.3f %.3f\n', acc(1), acc(2), acc(3));

fprintf('解算:%d次 总时间:%.3fs\n', N, N /Fs);
fprintf('最终误差(m): %.3f %.3f %.3f\n', pos(N, 1),  pos(N, 2),  pos(N, 3));


