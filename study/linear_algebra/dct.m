clear;
clc;


RGB = imread('autumn.tif');
I = rgb2gray(RGB);

% 把I 进行dct变换
J = dct2(I);


figure
% 打印J(频谱图)
imshow(log(abs(J)),[])
colormap(gca,jet(64))
colorbar

%去掉高频分量
J(abs(J) < 20) = 0;

%逆变换成时域图像
K = idct2(J);

figure
imshowpair(I,K,'montage')
title('Original Grayscale Image (Left) and Processed Image (Right)');

