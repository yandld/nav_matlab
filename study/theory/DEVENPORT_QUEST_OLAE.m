clear; clc; close all;
addpath('../../library');
%% https://www.coursera.org/learn/spacecraft-dynamics-kinematics/lecture/UFIBv/4-1-example-of-devenports-q-method
 

%% setuo the true attitude states;
theta_true = deg2rad([30 20 -10]);
BNtrue = angle2dcm(theta_true(1), theta_true(2), theta_true(3));

v1N = [1 0 0];
v2N = [0 0 1];

v1B_true = [BNtrue*v1N'];
v2B_true = [BNtrue*v2N'];

%% setup the measured attitude states
v1B = [0.8190 -0.5282 0.2243];
v2B = [-0.3138 -0.1584 0.9363];
% v1B = v1B / norm(v1B);
% v2B = v2B / norm(v2B);

% v1B = [0.8273 0.5541 -0.0920];
% v2B = [-0.8285 0.5522 -0.0955];
% v1N = [-0.1517 -0.9669 0.2050];
% v2N = [-0.8383 0.4494 -0.3044];


%% Devenport_Q mehold
w1 = 1;
w2 = 1;
W =[w1 w2];

vB = [v1B; v2B];
vB = vB.*W';
vN = [v1N; v2N];

%B = w1* v1B'*v1N + w2*v2B'*v2N;
B = vB'*vN;

S = B +B';
sigma = B(1,1) + B(2,2) + B(3,3);
Z = [B(2,3)-B(3,2) B(3,1)-B(1,3) B(1,2)-B(2,1)]';

K  = [sigma Z'; Z S - sigma*eye(3) ];

[V D] = eig(K);
[val index] = max(diag(D));
beta_q = V(:,index)';

DEVENPORT_Q = quat2dcm(beta_q)

%% OLAE  optimzal linear attitude estimator
W = eye(6);

d = [v1B - v1N,  v2B - v2N];

S =[ skew_symmetric(v1B + v1N); skew_symmetric(v2B + v2N)];
qBar = ((S'*W*S)^-1)  * S'*W*d';
OLAE_result = rod2dcm(qBar');
OLAE_result



