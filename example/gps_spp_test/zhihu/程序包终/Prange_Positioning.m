clear;
clc;
close all;

%----------------------------配置参数------------------------------------
tob = [01, 9, 4, 1, 30, 0];                                     %观测时刻的UTC时，年取后两位
geodeticHstation=93.4e-3;%NaN                            %测站大地高km，如果不知道填NaN
Alpha=[0.2235D-07  0.2235D-07 -0.1192D-06 -0.1192D-06];  %导航电文头文件中的电离层Alpha
Beta=[0.1290D+06  0.4915D+05 -0.1966D+06  0.3277D+06];   %导航电文头文件中的电离层Beta
filen='wuhn2470.01n';
fileo='wuhn2470.01o';
chooseTropo = 2;                                           %采用的对流层模型1:简化霍普菲尔德（Hopfield）改正模型 2:萨斯塔莫宁（Saastamoinen）改正模型
%------------------------------------------------------------------------
%----------------------------常数区--------------------------------------
cv = 299792458;                                            %光速m/s
a = 6378137;                                               %WGS84椭球半长轴m
finv = 298.2572236;                                        %WGS84椭球扁率倒数

%------------------------------------------------------------------------
%以下为计算过程
%1、在O文件中提取四颗以上卫星的C1观测值。
fprintf("读取 renix obs 数据...\r\n");
[obs, rec_xyz]  = read_rinex_obs(fileo);

% 取出当前GPST 时刻的所有观测数据
[~,tow] = UTC2GPST(tob(1),tob(2),tob(3),tob(4),tob(5),tob(6));
obs.data = obs.data([obs.data(:,2) == tow], :);

% 获得当前时刻观测数据的PRN，卫星个数和观测值
PRN = obs.data(:,3);
sv_num = numel(PRN);
C1 = obs.data(:,6);

fprintf("当前时刻: %d %d %d %d %d %d\r\n", tob(1), tob(2), tob(3), tob(4), tob(5), tob(6));
fprintf("卫星PRN:\r\n");
PRN'


% 读取N文件,并且找到和当前时刻最近的一份星历
fprintf("读取 renix nav 数据...\r\n");
all_eph = read_rinex_nav(filen);
for i = 1: sv_num
    one_sv_eph = all_eph([all_eph(:,1) == PRN(i)], :); % 读取某一个卫星的所有星历
    [~,idx] = min(abs(one_sv_eph(:, 17) - tow)); % 挑这一颗卫星里找与当前时间最接近的那一套星历
    eph(i,:) = one_sv_eph(idx, :);
end

% 计算所有卫星的钟差
for i=1:sv_num
    toc = eph(i, 20);
    a0 = eph(i, 21);
    a1 = eph(i, 22);
    a2 = eph(i, 23);
    sv_dt = sv_clock_bias(tow, toc, a0, a1, a2);
    deltat(i) = sv_dt;
end

%3、程序初始化，置测站概略位置为Xr，接收机钟差初值为dt。
X = [0 0 0 0]';
%4、选择epoch中一颗卫星Si，设其伪距为GSiC1
while 1
    
    for i=1:sv_num
        %5、计算卫星Si的卫星钟差dt
        %由计算卫星坐标时带出
        %6、计算卫星-接收机的近似几何距离Rs
        %（1）根据接收时间和伪距 计算信号发射时间
        tau(i) = C1(i)./cv;
        ttr(i) = tow-(tau(i) + deltat(i));
        
        %（2）计算发射时刻的卫星坐标 ，并对卫星坐标进行地球自转改正
        M0 = eph(i, 2);
        Delta_n = eph(i, 3);
        e = eph(i, 4);
        sqrtA = eph(i, 5);
        OMEGA = eph(i, 6);
        i0 = eph(i, 7);
        omega =  eph(i, 8);
        OMEGA_DOT = eph(i, 9);
        iDOT = eph(i, 10);
        Cuc = eph(i, 11);
        Cus = eph(i, 12);
        Crc = eph(i, 13);
        Crs = eph(i, 14);
        Cic = eph(i, 15);
        Cis = eph(i, 16);
        toe = eph(i, 17);
        toc = eph(i, 20);
        a0 = eph(i, 21);
        a1 = eph(i, 22);
        a2 = eph(i, 23);
        [Xs(i), Ys(i), Zs(i), deltat(i)] = ch_sat_pos(ttr(i), toc, a0, a1, a2, Crs, Delta_n, M0, Cuc, e, Cus, sqrtA, toe, Cic, OMEGA, Cis, i0, Crc, omega, OMEGA_DOT, iDOT);
        % [Xs(i), Ys(i), Zs(i), deltat(i)] = readatandcomp(filen, PRN(i), tob, Tems(i));
        spos = ch_sv_pos_rotate([Xs(i);Ys(i);Zs(i)], tau(i));
        
        %7、计算对流层延迟 dtrop
        dx = spos - X(1:3,1);
        [~, E1(i), ~] = topocent(X(1:3,1),dx);
        if isnan(geodeticHstation)
            [~,~,h] = togeod(a,finv,X(1,1),X(2,1),X(3,1));
            geodeticHstation=h*10^(-3);
        end
        if chooseTropo==1
            dtrop = tropo(sind(E1(i)),geodeticHstation,P0,T,e0,geodeticHstation,geodeticHstation,geodeticHstation);
        elseif chooseTropo==2
            dtrop = tropo_error_correction(E1(i),geodeticHstation);
        end
        % dtrop=0;%暂不考虑dtrop
        %8、计算电离层延迟 diono
        diono = Error_Ionospheric_Klobuchar(X(1:3,1)',[Xs(i);Ys(i);Zs(i)]', Alpha, Beta, tow);
        
        % diono=0;%暂不考虑dtrop
        l(i) = C1(i) + cv*deltat(i) - dtrop - diono+0;
        
        
        %10、求卫星Si方向余弦
        sv_pos(i,:) = spos;
    end
    % 11、选择Epoch中的下一颗卫星，设其伪距为ρS。
    % 12、重复5--11步，计算每颗卫星的系数和余数项。
    %13、将所有卫星的系数组成误差方程，以（x,y,z,cdtr）为未知数进行求解，形式为:AX=L
    L = l;
    
    [X, delta, G] = ch_gpsls(X,  sv_pos',  L);
    
    
    % P权阵
    % P=[sind(E1(1))^2,0,0,0,0,0;
    %     0,sind(E1(2))^2,0,0,0,0;
    %     0,0,sind(E1(3))^2,0,0,0;
    %     0,0,0,sind(E1(4))^2,0,0;
    %     0,0,0,0,sind(E1(5))^2,0;
    %     0,0,0,0,0,sind(E1(6))^2];
    % X=(inv(A'*P*A))*(A'*P*L);
    % Xi=X+X;%这一步重要
    % 15、与X0进行比较，判断位置差值。
    %    X = X + delta;
    if abs(delta(1,1))>0.001 || abs(delta(2,1))>0.001 || abs(delta(3,1))>0.001
    else
        break;
    end
end

% %16、输出满足条件的Xi。
GT = [-2267749.30600679, 5009154.2824012134, 3221290.677045021]';
residual = X(1:3) - GT;

fprintf("最终位置差: %f, %f, %f, 误差距离:%f\r\n", residual(1), residual(2), residual(3), norm(residual));

% ECEF转 LLA
[lat, lon, h] = ch_ECEF2LLA(X);

% 计算DOP
[VDOP, HDOP, ~, ~] = ch_gpsdop(G, lat, lon);

fprintf("坐标: lat:%f lon:%f\r\n", rad2deg(lat), rad2deg(lon));
fprintf("VDOP: %f  HDOP: %f\r\n", VDOP, HDOP);



