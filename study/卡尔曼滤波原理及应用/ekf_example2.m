%%
%  扩展Kalman滤波在目标跟踪中的应用
%  详细原理介绍及中文注释请参考：
%  《卡尔曼滤波原理及应用-MATLAB仿真》，电子工业出版社，黄小平著。
%% start
clc; clear; close all;

%% generate  data
T=1;
N=20/T;
X=zeros(4, N);
X(:,1)=[-100, 2, 200, 20];
Z=zeros(1,N);
delta_w=1e-3;
Q=delta_w*diag([0.5,1]) ; 
G=[T^2/2,0;T,0;0,T^2/2;0,T];
R=50;
F=[1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];
x0=200;
y0=300; 
Xstation=[x0, y0];
for t=2:N
    X(:,t)=F*X(:,t-1)+G*sqrtm(Q)*randn(2,1); % 此处有误，请修改为P87页一致即可运行
end
for t=1:N
    Z(t)=Dist(X(:,t),Xstation)+sqrtm(R)*randn;
end

Z = [309.6069  299.5920  305.5742  316.4845  297.8935  284.5903  294.6431  280.8063  280.1812  298.1703  291.7674  304.9453 319.3182  320.0148  332.6253  324.5630  351.3459  373.3350  374.3118  372.5059];

%% EKF
Xekf=zeros(4, N);
Xekf(:,1)=X(:,1);
P=eye(4);
Q = G * Q * G';

for i=2:N
    [Xekf(:,i), P] = ekf(Xekf(:,i-1), Z(i), R, Q, P, @fx, @hx);
end

    
%% plot
figure
hold on;box on;
plot(X(1,:),X(3,:),'-k.');
plot(Xekf(1,:),Xekf(3,:),'-r+');
legend('真实轨迹','EKF轨迹')

%% 计算误差
ErrEKF=zeros(1,N);
ErrMeasure=zeros(1,N);
for i=1:N
    real_dist = Dist(X(:,i),Xstation);
    ErrEKF(i)=abs(Dist(Xekf(:,i),Xstation) - real_dist);
    ErrMeasure(i) = abs(Z(i) - real_dist);
end

figure
hold on; box on;
plot(ErrEKF,'-ks','MarkerFace','r')
plot(ErrMeasure,'-ks','MarkerFace','b')
legend('EKF误差','测量误差');

%% support function
function d=Dist(X1,X2);
if length(X2)<=2
    d=sqrt( (X1(1)-X2(1))^2 + (X1(3)-X2(2))^2 );
else
    d=sqrt( (X1(1)-X2(1))^2 + (X1(3)-X2(3))^2 );
end
end
%% EKF 参考值
%  -100.0000    2.0000  200.0000   20.0000
%   -97.6672    2.1664  220.0894   20.0447
%   -95.8522    2.0250  240.0636   20.0163
%   -96.8106    1.1210  259.7056   19.9019
%   -95.9979    1.0475  279.5956   19.8989
%   -92.4483    1.5366  299.3195   19.8665
%   -91.7074    1.4050  319.3540   19.8934
%   -87.2093    1.8478  337.9989   19.7193
%   -82.7301    2.1801  355.9891   19.5067
%   -82.1972    1.9939  377.1590   19.6895
%   -79.3537    2.0809  395.4849   19.5541
%   -77.9466    2.0177  416.7000   19.7044
%   -77.0219    1.9223  441.1628   20.1001
%   -75.1467    1.9183  461.8010   20.1415
%   -73.2157    1.9182  484.1268   20.2977
%   -72.0054    1.8736  496.5607   19.7719
%   -69.7918    1.8940  518.9210   19.9345
%   -66.3964    1.9809  546.1280   20.3651
%   -64.5775    1.9720  565.9227   20.3332
%   -65.2114    1.8349  578.3789   19.9145

%% state transition
%     function for state transition, it takes a state variable Xn and returns 1) f(Xn) and 2) Jacobian of f at Xn. 
function [Val, Jacob] = fx(X)
T=1;
F=[1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];

Val = F*X;
Jacob = F;
end

%% function for measurement
%   function for measurement, it takes the state variable Xn and returns 1) g(Xn) and 2) Jacobian of g at Xn.
function [Val, Jacob] = hx(X)
    x0=200;
    y0=300; 
    Xstation=[x0, y0];

    Val = Dist(X, Xstation);
    Jacob=[(X(1,1)-x0)/Val, 0, (X(3,1)-y0)/Val, 0];
end
