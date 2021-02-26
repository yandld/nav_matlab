%% svd study 

clear all;
close all;
clc;

a=imread('svd_pic.jpg');
a = a(:,:,1); %取一个分量

imshow(mat2gray(a))
[m, n]=size(a);
title("原始图像");
fprintf("图片尺寸:%d x %d\n", m, n);


fprintf("原始图像大小:%d\n", m*n);
a=double(a);
r=rank(a);


[U, S, V]=svd(a);

k = 70; %修改这个值，取前k个主成分
%re=U*S*V';
U = U(:,1:k);
V = V(:,1:k);
S = S(1:k, 1:k);
re=U*S*V';

figure;
imshow(mat2gray(re));
title("压缩后图像");
fprintf("压缩后大小:%d\n", numel(U)  + k +  numel(V));

figure;
plot(diag(S));
title("奇异值")

