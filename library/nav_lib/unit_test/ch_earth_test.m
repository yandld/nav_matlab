%% NED_to_ECEF

clear;
clc;

lat = 0.785398;
lon = 0.523599;
ght = 1000;

Vned = [5 5 5]';
Cb2n = eye(3);

[ecef_pos, Vecef, Cb2e]  = ch_ned2ecef(lat, lon, ght, Vned, Cb2n);
  
  
%% ECEF_to_NED
clear;
clc;

ecef_pos = [-1890789.0  5194902.0  3170398.0]';
Vecef = [10 15 20]';
Cb2e = eye(3);


    
[lat, lon, ght, Vned,Cb2n] = ch_ecef2ned(ecef_pos,Vecef, Cb2e);


%% Radii_of_curvature test

[R_N,R_E]= ch_earth(deg2rad(45), 0);

%% Gravity_NED
lat = 0.785398163;
ght = 1000;
g = ch_gravity_ned(lat,ght);

%% Gravity_ECEF


ecef_pos = [4000000  2900000 4000000]';
    
g = ch_gravity_ecef(ecef_pos)



