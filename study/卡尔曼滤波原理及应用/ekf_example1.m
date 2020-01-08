%%
%  函数功能：标量非线性系统扩展Kalman滤波问题
%  状态函数：X(k+1)=0.5X(k)+2.5X(k)/(1+X(k)^2)+8cos(1.2k) +w(k)
%  观测方程：Z（k）=X(k)^2/20 +v(k)
%  详细原理介绍及中文注释请参考：
%  《卡尔曼滤波原理及应用-MATLAB仿真》，电子工业出版社，黄小平著。
%% start
clc; clear; close all;

%% 生成测试数据
T = 20;
Q = 10;
R = 1;
w = sqrt(Q)*randn(1,T);
v = sqrt(R)*randn(1,T);
x = zeros(1,T);
x(1) = 0.1;
y = zeros(1,T);

[y(1), ~] = hx(x(1));
y(1) = y(1) + v(1);
for k=2:T
    [x(k), ~] = raw_fx(x(k-1), k);
    x(k) = x(k)+ w(k-1);
    
   [y(k), ~] = hx(x(k));
    y(k) = y(k) + v(k);
end

y = [0.9244    1.1061    2.0635    0.7011    0.1730    8.5853    1.2883    1.8057    0.8002    0.3491    9.8035    4.0848 -1.0333    3.7694    0.1668    0.4596    0.7774   -1.6460    0.2518   -0.8959];

%% EKF
Xekf = zeros(1,T);
Xekf(1) = x(1);
P = ones(1);

for k=2:T
    fx = @(X, F) raw_fx(X, k);
	[Xekf(:, k), P] = ekf (Xekf(:, k-1), y(:,k), R, Q, P, fx, @hx);
end



%% 计算误差
ErrEKF=zeros(1,T);
ErrMeasure=zeros(1,T);
for k=1:T
    ErrEKF(k)=abs(Xekf(k)-x(k) );
    ErrMeasure(k) = abs(y(k) - x(k));
end

%% plot
figure
hold on;box on;
plot(x,'-ko','MarkerFace','g');
plot(Xekf,'-ks','MarkerFace','b');
plot(y,'-r+');
legend('真实值', 'EKF', '测量值');
figure
hold on;box on;
plot(ErrEKF,'-ko','MarkerFace','g');
plot(ErrMeasure,'-ks','MarkerFace','b');
legend('EKF误差', '测量值误差');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\

%% exkf 参考值：
%     0.1000
%    -4.9724
%    -7.3622
%    -3.5554
%     3.5311
%    14.1935
%     4.3959
%    -5.8394
%    -4.3123
%     3.3141
%    14.8252
%     9.0327
%    -0.7091
%    -8.6683
%     0.7886
%     5.4117
%     3.6772
%    -0.7234
%    -4.3960
%    -0.0536

%% state transition
%     function for state transition, it takes a state variable Xn and returns 1) f(Xn) and 2) Jacobian of f at Xn, also called F. 
function [Val, Jacob] = raw_fx(X, K)
X = 0.5*X+2.5*X/(1+X^2)+8*cos(1.2*K);
Jacob = 0.5+2.5 *(1-X^2)/(1+X^2)^2;
Val = X;
end

%% function for measurement
%   function for measurement, it takes the state variable Xn and returns 1)
%   g(Xn) and 2) Jacobian of g at Xn, also called H
function [Val, Jacob] = hx(X)
Val = X^2/20;
Jacob = X/10;
end
