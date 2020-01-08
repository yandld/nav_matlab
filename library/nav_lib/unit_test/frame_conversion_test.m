%% NED_to_ECEF

clear;
clc;

L_b = 0.785398;
lambda_b = 0.685398;
h_b = 0;

v_eb_n = [5 5 5]';
C_b_n = eye(3);

[r_eb_e,v_eb_e,C_b_e]  = ch_ned2ecef(L_b, lambda_b, h_b, v_eb_n, C_b_n);
  
  
%% ECEF_to_NED
clear;
clc;

r_eb_e = [-1890789.0  5194902.0  3170398.0]';
v_eb_e = [10 15 20]';
C_b_e = eye(3);


    
[L_b,lambda_b,h_b,v_eb_n,C_b_n] = ch_ecef2ned(r_eb_e,v_eb_e,C_b_e);


%% Radii_of_curvature test

[R_N,R_E]= Radii_of_curvature(0.78);

%% Gravity_NED
L_b = 0.785398163;
h_b = 1000;
g = ch_gravity_ned(L_b,h_b);

%% Gravity_ECEF


r_eb_e = [4000000  2900000 4000000]';
    
g = ch_gravity_ecef(r_eb_e)



