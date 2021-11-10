clear;
clc;
format short g;
close all;

X = randn(100000,1);
Y = X;

var(X+Y)
fprintf("var(X+Y): %f, var(X):%f\n", var(X+Y), var(X));


n = 10;
Z = mean(reshape(X,n,[]));
fprintf("平均%d次样本，VAR变为%f 倍\n", n, var(Z) / var(X));


