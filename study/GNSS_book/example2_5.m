clear;
clc
close all;

%% EXAMPLE 2.5(a)
 lat = deg2rad(45);
 lon = deg2rad(45);
 hgt = 0;
 
 vel_ned = [5 5 5]';
 a_ned = [5 0 0]';
 w_ned = [0.001 0 0]';
 
 

[R_meridian, R_transverse, Cecef2ned, Cecef2enu]= ch_earth(lat, lon, hgt);

Cned2ecef = Cecef2ned'

%% Velocity transformation
vel_ecef = Cned2ecef *vel_ned;
vel_ecef
%% Acceleration transformation
a_ecef = Cned2ecef*a_ned;
a_ecef
%% Angular rate transformation
w_ecef = Cned2ecef*w_ned;
w_ecef


%% EXAMPLE 2.5(b)
vel_ecef = [-8.53553391 -1.4644609 4.44089E-16]';
a_ecef = [-2.5 -2.5 3.535534]';
w_ecef = [-0.0005 -0.0005 0.000708]';
lat = deg2rad(45);
lon = deg2rad(45);
h = 0;
[~, ~, Cecef2ned, ~]= ch_earth(lat, lon, hgt);

%% Velocity transformation
v_ned = Cecef2ned*vel_ecef;
v_ned
%% Acceleration transformation
a_ned = Cecef2ned*a_ecef;
a_ned
%% Angular rate transformation
w_ned = Cecef2ned*w_ecef;
w_ned

