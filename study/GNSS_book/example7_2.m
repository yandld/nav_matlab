clear;
clc;
close all;

addpath('../../library/nav_lib'); 

%% INPUTS: INITIAL CONDITION
pos_user_true = [1000, 100]';
pos_user_inital = [0, 0]';
pos_user = pos_user_inital;
anchor_pos  = [0,1000; 0 -1000; 2000, 100]';
pr = vecnorm(anchor_pos - pos_user_true);

for i = 1:5
    pos_user = ch_triangulate(anchor_pos, pos_user, pr);
end








