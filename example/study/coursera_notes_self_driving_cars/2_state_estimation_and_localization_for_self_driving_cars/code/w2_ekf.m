clc;
clear;

R = 0.01;
Q = 0.1*eye(2);
u = -2;
delta = 0.5;
y = pi / 6;
x0 = [0 5]';
P0 = [0.01 0; 0 1];
S = 20;
D = 40;
F =  [1 delta; 0 1];

%  predict
x1 =F*x0 + [0 delta]'*u;
P1 = F*P0*F' + Q;

% update hx and Jacc: H
hx = atan(S / (D - x1(1) ));
H = [S / ((D - x1(1))^(2) + S^(2)), 0];

% update
K = P1*H'*(H*P1*H' +R )^(-1);
x1 = x1 + K*(y - hx);
P1 = (eye(2) - K*H)*P1;










 