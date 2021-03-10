clear;
clc;
close all;

A = [1 1 1 1; 2 5  7 8]';
B = [1 2 3 3]';

X = inv((A'*A))*A'*B

X = pinv(A)*B

[U,S,V] = svd(A);
PINV_A = V*pinv(S)*U';
X = PINV_A*B

X = U*U'*B


