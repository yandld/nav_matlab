clear;
clc;
close all;


% 一共进行n次，事件发生了k次，每次概率为p
% binopdf(k,n,p)
k = 45;
n = 100;
p = 0.5;

px = binopdf(k, n, p);

fprintf("使用binopdf： 概率为%d", px);

px = nchoosek(n,k) * p^(k) * (1-p)^(n-k);
fprintf("使用二项分布的公式计算，概率为%d", px);


x=1:1:100;
p=binopdf(x,100,0.5);
plot(x,p);
title('二项分布');
xlabel("100次试验发生x的次数"); ylabel("概率");


% 
% x=1:100;
% 
% %求最值
% p=binopdf(x,100,0.5);
% [ans,pos]=max(p);%取x=pos时，ans取最大



