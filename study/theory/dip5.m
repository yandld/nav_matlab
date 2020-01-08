clear all; clc;

%% moudle DIP5
syms  b1 b2 b3 real
syms  g1 g2 g3 real
syms  v1 v2 v3 real
syms  lambda real
syms norm_h norm_g real
B = [b1 b2 b3]';
V = [v1 v2 v3]';
G = [g1 g2 g3]';
% %G'*(V - B) - CONST

 f1= (V - B)' * (V - B) - norm_h^2;
 j1 = jacobian(f1, [B' norm_h lambda]);
 
 f2 = G' * (V - B) - norm_g * norm_h * lambda;
 j2 = jacobian(f2, [B' norm_h lambda]);


f1
j1

f2 
j2
