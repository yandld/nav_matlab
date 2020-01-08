%% 扩展Kalman滤波在纯方位目标跟踪中的应用实例
%  详细原理介绍及中文注释请参考：
%  《卡尔曼滤波原理及应用-MATLAB仿真》，电子工业出版社，黄小平著。
%% start
clc; clear; close all;

%% generate  data

T=1;
N=20/T;
X=zeros(4,N);
X(:,1)=[0,2,1400,-10];
Z=zeros(1,N);
delta_w=1e-4;
Q=delta_w * diag([1,1]) ;
G=[T^2/2 0; T 0; 0 T^2/2; 0 T];
R=0.1*pi/180;
F=[1 T 0 0; 0 1 0 0; 0 0 1 T; 0 0 0 1];
x0=0;
y0=1000; 
Xstation=[x0; y0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w=sqrtm(R)*randn(1,N);
for t=2:N
    X(:,t)=F * X(:,t-1) + G * sqrtm(Q) * randn(2,1);
end
for t=1 : N
    Z(t)=hfun(X(:,t),Xstation)+w(t);
end

Z = [1.5112    1.5605    1.5450    1.5576    1.4887    1.5747    1.5336    1.5187    1.5801    1.5408    1.5759 1.5498    1.4622    1.4638    1.5153    1.4336    1.4347    1.4049    1.4256    1.4362];

%% EKF
Xekf=zeros(4,N);
Xekf(:,1)=X(:,1);
P=eye(4);
Q = G*Q*G';

for i=2:N
	[Xekf(:,i), P] = ekf(Xekf(:,i-1), Z(i), R, Q, P, @fx, @hx);
end
    
%% 计算误差
for i=1:N
  ErrEKF(i)=sqrt(Dist(X(:,i),Xekf(:,i)));
end

%% plot 
figure
hold on;box on;
plot(X(1,:),X(3,:),'-k.');
plot(Xekf(1,:),Xekf(3,:),'-r+');
legend('真实轨迹','EKF轨迹')
figure
hold on; box on;
plot(ErrEKF,'-ks','MarkerFace','r')
title('EKF 误差');
figure 
hold on;box on;
plot(Z/pi*180,'-r.','MarkerFace','r');
plot(Z/pi*180+w/pi*180,'-ko','MarkerFace','g');
legend('真实角度','观测角度');

%% support function

% 输入两个坐标，返回角度
function cita=hfun(X1,X0)
if X1(3,1)-X0(2,1)>=0
    if X1(1,1)-X0(1,1)>0
        cita=atan(abs( (X1(3,1)-X0(2,1))/(X1(1,1)-X0(1,1)) ));
    elseif X1(1,1)-X0(1,1)==0
        cita=pi/2;
    else
        cita=pi/2+atan(abs( (X1(3,1)-X0(2,1))/(X1(1,1)-X0(1,1)) ));
    end
else
    if X1(1,1)-X0(1,1)>0
        cita=3*pi/2+atan(abs( (X1(3,1)-X0(2,1))/(X1(1,1)-X0(1,1)) ));
    elseif X1(1,1)-X0(1,1)==0
        cita=3*pi/2;
    else
        cita=pi+atan(abs( (X1(3,1)-X0(2,1))/(X1(1,1)-X0(1,1)) ));
    end
end
end

function d=Dist(X1,X2)
    if length(X2)<=2
        d=( (X1(1)-X2(1))^2 + (X1(3)-X2(2))^2 );
    else
        d=( (X1(1)-X2(1))^2 + (X1(3)-X2(3))^2 );
    end
end

%% state transition
%     function for state transition, it takes a state variable Xn and returns 1) f(Xn) and 2) Jacobian of f at Xn. 
function [Val, Jacob] = fx(X)
T=1;
F=[1 T 0 0; 0 1 0 0; 0 0 1 T; 0 0 0 1];

Val = F * X;
Jacob = F;
end

%% function for measurement
%   function for measurement, it takes the state variable Xn and returns 1) g(Xn) and 2) Jacobian of g at Xn.
function [Val, Jacob] = hx(X)
    x0=0;
    y0=1000; 
    Xstation=[x0;y0];

    Val = hfun(X,Xstation);
    D = Dist(X, Xstation);
    Jacob = [-(X(3,1)-y0)/D,0,(X(1,1)-x0)/D,0];
end
%% 参考数据

%    1.0e+03 *
% 
%          0    0.0020    1.4000   -0.0100
%     0.0020    0.0020    1.3900   -0.0100
%     0.0041    0.0021    1.3800   -0.0100
%     0.0061    0.0020    1.3700   -0.0100
%     0.0096    0.0024    1.3600   -0.0100
%     0.0107    0.0021    1.3500   -0.0100
%     0.0128    0.0021    1.3400   -0.0100
%     0.0152    0.0022    1.3300   -0.0100
%     0.0138    0.0017    1.3202   -0.0100
%     0.0144    0.0016    1.3103   -0.0100
%     0.0124    0.0012    1.3006   -0.0099
%     0.0121    0.0011    1.2907   -0.0099
%     0.0168    0.0014    1.2806   -0.0099
%     0.0205    0.0016    1.2703   -0.0100
%     0.0205    0.0015    1.2608   -0.0099
%     0.0245    0.0016    1.2501   -0.0100
%     0.0274    0.0017    1.2395   -0.0100
%     0.0308    0.0018    1.2281   -0.0101
%     0.0325    0.0018    1.2182   -0.0101
%     0.0333    0.0018    1.2096   -0.0100
%% End Of Scripts
