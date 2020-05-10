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

    pos_user = ch_multilateration(anchor_pos, pos_user , pr);









