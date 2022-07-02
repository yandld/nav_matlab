clear;
clc
close all;
addpath('../../library/nav_lib');

%% 用户真实坐标及接受机钟差(m)
true_user_states = [4245849, -2451342, 4113840, 1000000]';

%% 卫星位置
sat = zeros(3,5);
sat(:,1) = [21630742.37 -7872946.37 13290000]';
sat(:,2) = [9799722.428 -11678854.4 21773061.34]';
sat(:,3) = [15014045.82 2647381.37 21773061.34]';
sat(:,4) = [17020279.96 -20283979.8 2316599.642]';
sat(:,5) = [26076581.77 4598004.93 2316599.642]';
pos = zeros(4,1);
dp = 0;

%% presduo range
pr = vecnorm(sat - true_user_states(1:3)) + true_user_states(4) ;
[pos, dp, G] = ch_gpsls(pos, sat,  pr);

pos
dp



