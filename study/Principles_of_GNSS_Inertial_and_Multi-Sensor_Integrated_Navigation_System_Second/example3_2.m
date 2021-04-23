clear;
clc
close all;

%%
% EXAMPLE 3.2 (Chapter 3 Example B)
%Kalman filter estimating 2D position

%% INTPUS 
X = [ 1; 0];
P = [0.25, 0.1; 0.1, 0.25];
F = [1 0; 0 1];
Svx =1.8;
Svy = 2.2;
T = 0.5;
Q = [Svx*T, 0; 0, Svy*T];
H = [1 0; 0 1];
R = [1, 0.1; 0.1, 1];
Z = [2; -2];

%predict
X = F*X;
P = F*P*F' + Q;


%correct
K = P*H'*(H*P*H' + R)^(-1);
X = X + K*(Z - H*X);
P = (eye(2) - K*H)*P;







