clear;
clc
close all;

%% EXAMPLE 2.6(a)
 lat = deg2rad(45);
 lon = deg2rad(30);
 hgt = 100;

[R_meridian, R_transverse, Cecef2ned, Cecef2enu]= ch_earth(lat, lon, hgt);

xyz = ch_lla2ecef(lat, lon, hgt);

xyz

%% EXAMPLE 2.6(b)





