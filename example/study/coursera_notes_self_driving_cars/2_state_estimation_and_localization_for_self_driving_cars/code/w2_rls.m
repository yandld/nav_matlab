clc;
clear;

I = [0.2 0.3 0.4 0.5 0.6];
V = [1.23 1.38 2.06 2.47 3.17];


 % batch soluction
 H = [I', ones(5, 1)];
 
 x_ls = (H'*H)^(-1)*H'*V';
 
 
% Recursive Estimator
P = diag([10^(2), 0.2^(2)]);
x = [9 0]';

% record history of state varible 
x_hist(1,:) = x; 

num_meas = length(I);
R = 0.0225;

for i = 1:num_meas
    H = [I(i), 1];
    K = P*H'*(H*P*H' + R)^(-1);
    x = x + K*(V(i) - H*x);
    P = (eye(2) - K*H)*P;
    x_hist(i,:) = x;
end

plot(I,V, '.-')
hold on;
grid on;
plot(I, I*x_ls(1) + x_ls(2) )
hold on;
plot(I, x_hist(:,1)'.*I +  x_hist(:,2)', '*')
legend('measurement','LS','RLS');
ylabel('V')
xlabel('I')
title('LS&RLS')






 