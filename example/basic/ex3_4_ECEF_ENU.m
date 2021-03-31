clear;
clc;
close all;
format 

lat0 = deg2rad(45.9132);
lon0 = deg2rad(36.7484);
h0 = 1877753.2;

x = 5507528.9;
y = 4556224.1;
z = 6012820.8;

[E, N, U] = ch_ECEF2ENU([x y z], lat0, lon0, h0);
fprintf("EX1 - 已知起始点LLA 和 目标点的ECEF坐标， 求ENU系下偏移量\n");
fprintf("ENU偏移量: %f %f %f\n\n", E, N, U);



E = 355601.261550;
N = -923083.155899;
U = 1041016.423793;

 XYZ = ch_ENU2ECEF(E, N, U, lat0, lon0, h0);
fprintf("EX2 - 已知起始点LLA 和 目标点的ENU偏移量， 求目标点ECEF坐标\n");
fprintf("ECEF坐标: %f %f %f\n", XYZ(1), XYZ(2), XYZ(3));


