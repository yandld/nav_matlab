clear; clc; close all;
addpath('../../library');

%% https://www.coursera.org/learn/spacecraft-dynamics-kinematics/lecture/upoxw/14-crps-through-cayley-transform

dcm = [0.8138    0.2962   -0.5000; 0.2359    0.6179    0.7500; 0.5311   -0.7283    0.4330];

rod = dcm2rod(dcm);

dcm 

rod

% do cayley transform
rod2 = (eye(3) - dcm) * (eye(3) + dcm)^-1;

rod2 = [rod2(3,2) rod2(1,3) rod2(2,1)]

    
    
