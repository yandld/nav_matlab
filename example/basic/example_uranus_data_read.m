clear;
clc;
close all;

%% 读取数据
data = csvread("UranusData.csv", 1, 1);
gyroReading = data(:,4:6) / 10;




plot(gyroReading);


 z = gyroReading(:,3);
 N = length(z);
 
sqrt( (dot(z, z) - sum(z)^(2) / N) / (N - 1) )

gstd =  std(gyroReading)*1;

fprintf("静止陀螺std:  X:%.3f, Y:%.3f, Z:%.3f\n", gstd(1), gstd(2), gstd(3) );

%保存为文本文件
% save static_gyr.txt gyroReading -ascii
