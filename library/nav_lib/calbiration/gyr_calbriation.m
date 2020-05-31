%% Start of script
close all;                         % close all figures
clear;                              % clear all variables
clc;                                 % clear the command terminal
addpath('../library');      % include quaternion library

%% test data
A = [
   100  0       5
   0    100     0
   0    0     100
-100  0        -5
  0  -100      0
  0   0     -100
    ];

bias =  [1 ,2 ,3];
% add biasl
A = A + bias;

theory = [
    50   0    0
    0   50    0
    0    0    50
 -50   0     0
   0   -50   0
  0     0    -50
    ];


% A = [
%    65.2191    0.1712    0.4618
%    -0.0901   71.9079   -0.0728
%    -0.5425   -0.1436   71.7273
%   -65.0873   -0.1008   -0.4814
%     0.1573  -71.9432    0.0436
%     0.6128    0.2009  -71.7076
%     ];
% 
% bias =  [0.0320   0.0326  -0.0103];
% theory = [65.5618  0 0; 0 72.0649  0; 0 0 72.1298 ; -65.5081 0 0; 0 -72.1298 0; 0 0 -72.0649];


%% Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes 

% Ab: A minus bias 

Ab = A - bias;
Ab = Ab(1:3, :)';
Bt = theory(1:3,:);
mis1 = Bt / Ab;
mis1


%% LS mehold
Ab = A - bias;

% apply least square
X = (Ab'*Ab)^(-1) * Ab'*theory;

mis2 = X'
