addpath('../../library');

clc; clear; close all;

T=1;
N=10/T;
X=zeros(4,N);
X(:,1)=[-100, 2, 200, 20];
Z=zeros(2,N);
Z(:,1)=[X(1,1),X(3,1)];
delta_w=1e-2;
Q=delta_w*diag([0.5,1,0.5,1]) ;
R=10*eye(2);
F=[1,T,0,0; 0,1,0,0; 0,0,1,T; 0,0,0,1];
H=[1,0,0,0; 0,0,1,0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=2:N
    X(:,t)=F*X(:,t-1)+sqrtm(Q)*randn(4,1);
    Z(:,t)=H*X(:,t)+sqrtm(R)*randn(2,1);  % 此处有误，请修改为P54页第二句一致即可运行
end

Z = [ -100.0000 -102.1812  -96.9707  -83.9352  -92.0613  -85.9386  -99.1500  -81.1104  -76.5856  -86.2980;  200.0000  205.0092  237.8325  264.7653  297.9099  302.8866  334.6368  317.3201  370.0130  365.7210];

  
Xkf=zeros(4, N);
Xkf(:,1)=X(:,1);
P = ones(4);

for i=2:N
   [ Xkf(:, i) , P] = kf(Xkf(:, i-1), Z(:, i), F, H, R ,Q, P);
end


for i=1:N
    Err_Observation(i)=RMS(X(:,i),Z(:,i));
    Err_KalmanFilter(i)=RMS(X(:,i),Xkf(:,i));
end
figure
hold on;box on;
plot(X(1,:),X(3,:),'-k');
plot(Z(1,:),Z(2,:),'-b.');
plot(Xkf(1,:),Xkf(3,:),'-r+');
legend('真实轨迹','观测轨迹','滤波轨迹')
figure
hold on; box on;
plot(Err_Observation,'-ko','MarkerFace','g')
plot(Err_KalmanFilter,'-ks','MarkerFace','r')
legend('滤波前误差','滤波后误差')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dist=RMS(X1,X2);
if length(X2)<=2
    dist=sqrt( (X1(1)-X2(1))^2 + (X1(3)-X2(2))^2 );
else
    dist=sqrt( (X1(1)-X2(1))^2 + (X1(3)-X2(3))^2 );
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
