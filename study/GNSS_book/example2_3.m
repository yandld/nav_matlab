clear;
clc
close all;


%% EXAMPLE 2.3 Calculation of Gravity at various latitudes and heights
lat = deg2rad(45);
lon = 0;
hgt = 10000;

[~, ~, ~, ~, gravity]= ch_earth(lat, lon, hgt);
gravity