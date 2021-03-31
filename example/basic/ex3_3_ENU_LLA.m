clear;
clc;
close all;
format 


lat0 = deg2rad(46.017);
lon0 = deg2rad(7.750);
h0 = 1673;

lat = deg2rad(45.976);
lon = deg2rad(7.658);
h = 4531;

[E, N, U] = ch_LLA2ENU(lat, lon, h, lat0, lon0, h0);

fprintf("EX1 - 已知原点LLA: %f %f %f, 目标点LLA: %f %f %f \n", lat0, lon0, h0, lat, lon, h);
fprintf(" 转换后得到ENU增量:%f %f %f\n", E, N, U);


lat0 = deg2rad(46.017);
lon0 = deg2rad(7.750);
h0 = 1673;

xEast = -7134.8;
yNorth = -4556.3;
zUp = 2852.4;

[lat, lon, h] = ch_ENU2LLA(xEast, yNorth, zUp, lat0, lon0, h0);
fprintf("EX2 - 已知原点LLA: %f %f %f, ENU系下增量:%f %f %f \n", lat0, lon0, h0, xEast, yNorth, zUp);
fprintf("转换得到目标点LLA:%f(deg) %f(deg) %f(m)\n", rad2deg(lat), rad2deg(lon), h);


