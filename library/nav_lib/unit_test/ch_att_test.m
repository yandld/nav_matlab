
%%
addpath('../')
clear;
clc;


%% dv2att
vn1 = [0.0000   0.0000   1.0000 ]';
 vn2 =[26.3883   0.0000 -58.3877 ]';
vb1 = [0.1736   0.5318   0.8191 ]';
vb2 = [12.9850 -43.5063 -45.2112]';

[qnb, att, Cnb] = ch_dv2atti(vn1, vn2, vb1, vb2);

% 
% %% att test
% %Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2nd Edition, by Paul D. Groves
% 
% %Example 2.1(a)
% %  Set of Euler angles describing rotation from local navigation frame to body frame
% r = -30; p = 30; y =45;
% eul = deg2rad([r p y]);
% Cn2b = ch_eul2m(eul);
% Cb2n = Cn2b';
% 
% Qn2b = ch_eul2q(eul);
% 
% 
% %% Example2.1(b)
% Cb2n = [0.612372 -0.78915 -0.04737; 0.612372 0.435596 0.65974; -0.5 -0.43301 0.75];
% Cb2n * Cn2b;
% eul = ch_m2eul(Cn2b);
% Qn2b = ch_m2q(Cn2b);
% 
% %% Example2.1(c)
% Qn2b = [0.836356 -0.32664 0.135299 0.418937];
% Cn2b = ch_q2m(Qn2b);
% eul = ch_q2eul(Qn2b);
% 
% Qb2n = quatconj(Qn2b);
% 
% test_v = [0 0 1]';
% 
% 
% 
%   