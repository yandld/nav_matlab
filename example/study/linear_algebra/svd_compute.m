clear;
clc;

A = [4 4; -3 3];

fprintf("matlab SVD分解\n");

[U,S,V] = svd(A)


fprintf("A'A的特征值\n");

[V,D] = eig(A'*A)


fprintf("AA'的特征值\n");

[V,D] = eig(A*A')

