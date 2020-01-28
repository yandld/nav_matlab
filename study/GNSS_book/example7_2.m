clear;
clc
close all;

addpath('../../library/nav_lib'); 

%% INPUTS: INITIAL CONDITION
pos_user_true = [1000, 100]';
pos_user_guass = [0, 0]';
pos_user = pos_user_guass;
pos_base  = [0,1000; 0 -1000; 2000, 100]';
r_measure = vecnorm(pos_base - pos_user_true);

for i = 0:4
    Ra = vecnorm(pos_base - pos_user);
    H(1,:) = -(pos_base(:,1) - pos_user)/Ra(1)';
    H(2,:) = -(pos_base(:,2) - pos_user)/Ra(2)';
    H(3,:) = -(pos_base(:,3) - pos_user)/Ra(3)';
    
    pos_user = pos_user + inv(H'*H)*H'*(r_measure - Ra)'
end







