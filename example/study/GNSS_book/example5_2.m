clear;
clc
close all;

addpath('../../library/nav_lib'); 

%% INPUTS: INITIAL CONDITION
p0 = [0; 0; 6356760];
v0 = [0;  0; 0];
att0 = deg2rad([0; 0; 30]);

%% INPUTS: INITAL MEASUREMENT
ts = 0.01
gyr = [0; 0; 7.29E-5];
acc = [2; 0; 9.832];

%% ATTITUDE UPDATE
Cnb_last = eul2dcm(att0)'  % B->N
Cnb = Cnb_last * (eye(3) + skew_symmetric(gyr)*ts);

%% Specific Force Frame Transforation
f = 0.5*(Cnb_last + Cnb)*acc;


%%  Update Velocity
a = f + [0  0  -9.832]';
v_last = v0;
v = v_last + a*ts;

%% UPDATE POSITION
p = p0 + (v_last + v) * ts / 2;

p


