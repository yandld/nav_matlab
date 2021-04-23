clear;
clc
close all;


%% INPUTS: INITIAL CONDITION
pos_user_true = [1000, 100]';
pos_user_inital = [100, 0]';
pos_user = pos_user_inital;
anchor_base  = [0,1000; 0 -1000]';
pr = vecnorm(anchor_base - pos_user_true);

for i = 1:5
    pos_user = ch_triangulate(anchor_base, pos_user, pr);
end








