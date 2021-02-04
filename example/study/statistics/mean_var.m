clear;
clc;

data = rand(10, 1);

fprintf("matlab 自带函数 均值:%f ,方差:%f\n", mean(data), var(data));


N = length(data);

% https://blog.csdn.net/u014485485/article/details/77679669
m = 0;
last_m = 0;
F = 0;
for i = 1:N
    m = m + (data(i) - m) / i;
    F = F + (data(i) - last_m) * (data(i) - m);
     last_m = m;
end
    var2 = F / (N-1);
    
fprintf("递推法:  均值:%f ,方差:%f\n", m, var2);

for i = 1:N
    m = m + (data(i) - m) / i;
    F = F + (data(i) - last_m) * (data(i) - m);
     last_m = m;
end


sum = 0;
sum_sqrt = 0;
for i = 1:N
    sum  = sum + data(i);
    sum_sqrt =  sum_sqrt + data(i)^(2);   
end

m = sum / N;
var2 = sum_sqrt - sum^(2) / N;
var2 = var2 / (N-1);

fprintf("另外一种递推法:  均值:%f ,方差:%f\n", m, var2);



