clear;
clc;
close all;
format 

latd = 45;
lond = 30;
h = 1000;

lat = deg2rad(latd);
lon = deg2rad(lond);

fprintf("EX1: 已知经纬高:%.4f° %.4f° %.2fm\n", latd, lond ,h);

 XYZ= ch_LLA2ECEF(lat, lon, h);
fprintf("转换为ECEF系坐标为:%.3f(m), %.3f(m) ,%.3f(m)\n\n", XYZ);



XYZ = [3912960.837, 2259148.993, 4488055.516];
fprintf("EX2: 已知ECEF坐标:%.3f(m) %.3f(m) %.3f(m)\n", XYZ);
[lat, lon, h] = ch_ECEF2LLA(XYZ);
fprintf("转换为经纬高(LLA):%f(rad) %f(rad) %f(m)\n", lat, lon, h);



