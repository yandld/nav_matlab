clear;
clc
close all;

%%
% EXAMPLE 3.1 (Chapter 3 Example A)
%Kalman filter estimating single-axis position and velocity

%% INTPUS 
X = [0; 2];
P = [1 0.1; 0.1, 0.25];
F = [1 0.5; 0 1];
Sa = 0.2;
T = 0.5;
Q = [Sa*T^(3)/3, Sa*T^(2)/2; Sa*T^(2)/2, Sa*T];
H = [1 0];
R = 2.25;
Z = 2;

% predict

X = F*X;
P = F*P*F' + Q;

% correct
K = P*H'*(H*P*H' + R)^(-1);
X = X + K*(Z - H*X);
P = (eye(2) - K*H)*P;


