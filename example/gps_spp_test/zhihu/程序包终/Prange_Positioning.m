clear;
%----------------------------配置参数------------------------------------
tob = [01,9,4,9,30,0];                                     %观测时刻的UTC时，年取后两位
geodeticHstation=93.4e-3;%NaN                            %测站大地高km，如果不知道填NaN
Alpha=[0.2235D-07  0.2235D-07 -0.1192D-06 -0.1192D-06];  %导航电文头文件中的电离层Alpha
Beta=[0.1290D+06  0.4915D+05 -0.1966D+06  0.3277D+06];   %导航电文头文件中的电离层Beta
filen='wuhn2470_clearhdr.01n';                           %去头的n文件
fileo='wuhn2470_clearhdr.01o';                           %去头的o文件
filem='wuhn2470_clearhdr.01m';                           %去头的m文件
chooseTropo=2;                                           %采用的对流层模型1:简化霍普菲尔德（Hopfield）改正模型 2:萨斯塔莫宁（Saastamoinen）改正模型
%------------------------------------------------------------------------
%----------------------------常数区--------------------------------------
omegaE=7.29211511467e-5;                                 %地球自转角速度rad/s
cv=299792458;                                            %光速m/s
a=6378137;                                               %WGS84椭球半长轴m
finv=298.2572236;                                        %WGS84椭球扁率倒数

%------------------------------------------------------------------------
%以下为计算过程
%1、在O文件中提取四颗以上卫星的C1观测值。
[PRN, C1]=readfileo(fileo,tob(4),tob(5),tob(6));%去头的o文件
sv_num = length(PRN(:));
%2、在N文件中提取对应卫星的数据。
[~,GPST] = UTC2GPST(tob(1),tob(2),tob(3),tob(4),tob(5),tob(6));

for i=1:sv_num
    [~,~,~,deltat(i)]=readatandcomp(filen,PRN(i),tob);
end
%3、程序初始化，置测站概略位置为Xr，接收机钟差初值为dt。
X0 = [0 0 0 0]';
%4、选择epoch中一颗卫星Si，设其伪距为GSiC1
 while 1   

     for i=1:sv_num
%5、计算卫星Si的卫星钟差dt
%由计算卫星坐标时带出
%6、计算卫星-接收机的近似几何距离Rs
%（1）根据接收时间和伪距 计算信号发射时间
        tau(i)=C1(i)./cv;
        Tems(i) = GPST-(tau(i) + deltat(i));
%（2）计算发射时刻的卫星坐标 ，并对卫星坐标进行地球自转改正
        [Xs(i),Ys(i),Zs(i),deltat(i)] = readatandcomp(filen,PRN(i),tob,Tems(i));
        Prec = ch_sv_pos_rotate([Xs(i);Ys(i);Zs(i)], tau(i));
        
%（3）计算近似几何距离
        Rs=sqrt((Prec(1,1)-X0(1,1))^2+(Prec(2,1)-X0(2,1))^2+(Prec(3,1)-X0(3,1))^2);

%7、计算对流层延迟 dtrop
        dx = Prec - X0(1:3,1);
        [~, E1(i), ~] = topocent(X0(1:3,1),dx);
        if isnan(geodeticHstation)
           [~,~,h] = togeod(a,finv,X0(1,1),X0(2,1),X0(3,1)); 
           geodeticHstation=h*10^(-3);
        end
        if chooseTropo==1
            dtrop = tropo(sind(E1(i)),geodeticHstation,P0,T,e0,geodeticHstation,geodeticHstation,geodeticHstation);
        elseif chooseTropo==2
            dtrop = tropo_error_correction(E1(i),geodeticHstation);
        end
% dtrop=0;%暂不考虑dtrop
%8、计算电离层延迟 diono
        diono=Error_Ionospheric_Klobuchar(X0(1:3,1)',[Xs(i);Ys(i);Zs(i)]', Alpha, Beta, GPST);
        
% diono=0;%暂不考虑dtrop
%9、求卫星Si在观测方程中的余数项
        l(i)=C1(i)-Rs+cv*deltat(i)-dtrop-diono+0;
%10、求卫星Si方向余弦
        b0(i)=(X0(1,1)-Prec(1,1))./Rs;
        b1(i)=(X0(2,1)-Prec(2,1))./Rs;
        b2(i)=(X0(3,1)-Prec(3,1))./Rs;
        b3(i)=1;
    end
% 11、选择Epoch中的下一颗卫星，设其伪距为ρS。
% 12、重复5--11步，计算每颗卫星的系数和余数项。
%13、将所有卫星的系数组成误差方程，以（x,y,z,cdtr）为未知数进行求解，形式为:AX=L
    A=[b0',b1',b2',b3'];L=l';
%14、求解： X(i)=(inv(A'*P*A))*(A'*P*L)，得出定位结果。
    X=(inv(A'*A))*(A'*L);
    V=A*X-L;
% P权阵
% P=[sind(E1(1))^2,0,0,0,0,0;
%     0,sind(E1(2))^2,0,0,0,0;
%     0,0,sind(E1(3))^2,0,0,0;
%     0,0,0,sind(E1(4))^2,0,0;
%     0,0,0,0,sind(E1(5))^2,0;
%     0,0,0,0,0,sind(E1(6))^2];
% X=(inv(A'*P*A))*(A'*P*L);
    Xi=X0+X;%这一步重要
% 15、与X0进行比较，判断位置差值。
    if abs(X(1,1))>0.001||abs(X(2,1))>0.001||abs(X(3,1))>0.001
       X0=Xi;
    else
       X0=Xi;
       break;
    end
end
% %16、输出满足条件的Xi。
Xr = X0(1:3,1)
deltax = X0(1:3,1)-[-2267749.30600679;5009154.2824012134;3221290.677045021]
deltas = sqrt(deltax(1,1)^2+deltax(2,1)^2+deltax(3,1)^2)
% E1
% P=zeros(SatNum,SatNum);
% for i=1:SatNum
%     P(i,i)=sind(E1(i))^2;
% end
% Qx=inv(A'*P*A);
% GDOP=sqrt(Qx(1,1)+Qx(2,2)+Qx(3,3)+Qx(4,4))
% PDOP=sqrt(Qx(1,1)+Qx(2,2)+Qx(3,3))
% TDOP=sqrt(Qx(4,4))
% HDOP=sqrt(Qx(1,1)^2+Qx(2,2)^2+Qx(3,3)^2+Qx(4,4)^2)
% VDOP=sqrt(Qx(1,1)^2+Qx(2,2)^2+Qx(3,3)^2+Qx(4,4)^2)