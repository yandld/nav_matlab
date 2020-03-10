clear;
clc
close all;


%% EXAMPLE 2.2(a) Conversion of Curvilinear Position to Cartesian ECEF Position

lat = deg2rad(45);
lon = deg2rad(30);
hgt = 1000;

[R_meridian, R_transverse, ~, ~, ~]= ch_earth(lat, lon, hgt);

xyz = ch_lla2ecef(lat, lon, hgt);
xyz


%% EXAMPLE 2.2(b) Conversion of Cartesian ECEF Position to Curvilinear Position
xyz = [3912960.837 2259148.993 4488055.516]';
[lat lon hgt] = ch_ecef2lla(xyz);
lat 
lon 
hgt





