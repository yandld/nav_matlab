clear;
clc
close all;
addpath('../../library/nav_lib');

%% inital state
SV_pos_ecef = [26580000, 0, 0]';
SV_vel_evef = [0 1755 3170]';
lat = 0.785398;
lon =  0.174533;
ght = 1000;
R0 = 6378137;
e = 0.0818191908425;
omaga = 7.29E-05;
c = 299792458;

[~, RE ] = ch_earth(lat, ght);

usr_ecef = ch_lla2ecef(lat, lon, ght);
 
 [az, el] = satellite_az_el(SV_pos_ecef, usr_ecef);
 rad2deg(el)
 rad2deg(az)
 

