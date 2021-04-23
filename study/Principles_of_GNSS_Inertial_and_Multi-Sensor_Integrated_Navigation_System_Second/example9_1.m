clear;
clc
close all;
addpath('../../library/nav_lib');

%% inital state
true_user_states = [4245849 -2451342 4113840, 1000000]';

station = zeros(3,5);
station(:,1) = [21630742.37 -7872946.37 13290000]';
station(:,2) = [9799722.428 -11678854.4 21773061.34]';
station(:,3) = [15014045.82 2647381.37 21773061.34]';
station(:,4) = [17020279.96 -20283979.8 2316599.642]';
station(:,5) = [26076581.77 4598004.93 2316599.642]';
pos = zeros(3,1);
bias = 0;

%% presduo range
pr = vecnorm(station - true_user_states(1:3)) + true_user_states(4) ;
[pos, bias, dp, G] = ch_gpsls(pos, bias, station,  pr);

pos
bias



