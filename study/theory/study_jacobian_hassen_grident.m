%% https://blog.csdn.net/jinshengtao/article/details/51615162

%% Start of script
addpath('../../library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                 % clear the command terminal

syms x1 x2 y
f = (3/2)*x1*x1 + (1/2)*x2*x2 - x1*x2 - 2*x1;
%f = (1-x1)^2+100*(x2-x1^2)^2;

H = hessian(f,[x1 x2]);
G = gradient(f, [x1 x2]);
J = jacobian(f, [x1 x2]);

H
G
J

%% another example of J
syms r l f
x=r*cos(l)*cos(f);
y=r*cos(l)*sin(f);
z=r*sin(l);
J = jacobian([x;y;z],[r l f]);



