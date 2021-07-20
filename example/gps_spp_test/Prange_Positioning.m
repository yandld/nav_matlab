clear;
clc;
close all;

% 参考: https://www.telesens.co/2017/07/17/calculating-position-from-raw-gps-data/#Rotating_the_Satellite_Reference_Frame
% GPS原理及应用 谢刚


%tob = [01, 9, 4, 0, 30, 0];                                     %观测时刻的UTC时，年取后两位

geodeticHstation=93.4e-3;%NaN                            %测站大地高km，如果不知道填NaN
Alpha = [0.2235D-07  0.2235D-07 -0.1192D-06 -0.1192D-06];  %导航电文头文件中的电离层Alpha
Beta = [0.1290D+06  0.4915D+05 -0.1966D+06  0.3277D+06];   %导航电文头文件中的电离层Beta
filen='wuhn2470.01n';
fileo='wuhn2470.01o';

%  filen='rtcm_data.nav';
%  fileo='rtcm_data.obs';

chooseTropo = 2;                                           %采用的对流层模型1:简化霍普菲尔德（Hopfield）改正模型 2:萨斯塔莫宁（Saastamoinen）改正模型

cv = 299792458;                                            %光速m/s
a = 6378137;                                               %WGS84椭球半长轴m
finv = 298.2572236;                                        %WGS84椭球扁率倒数

%------------------------------------------------------------------------
%以下为计算过程
%1、在O文件中提取四颗以上卫星的C1观测值。
fprintf("读取 renix obs 数据...\r\n");
%[obs, ~]  = read_rinex_obs(fileo);
load 'obs.mat';


% 读取N文件,并且找到和当前时刻最近的一份星历
fprintf("读取 renix nav 数据...\r\n");
all_eph = read_rinex_nav(filen);
fprintf("读取完成\r\n");


fprintf("开始定位...\r\n");

%3、程序初始化，置测站概略位置为Xr，接收机钟差初值为dt。
X = [0 0 0 0]';

epochs = unique(obs.data(:, obs.col.TOW));
TimeSpan=epochs(1:200);

for ii = 1:length(TimeSpan);
    % 提取当前TOW时刻的所有OBS
    this_TOW = TimeSpan(ii);
    index = find(obs.data(:,obs.col.TOW) == this_TOW);
    curr_obs.data = obs.data(index, :);
    curr_obs.col = obs.col;
    
    sv_num = size(curr_obs.data,1);

    if (sv_num < 4)
        continue;
    end
    
	sv_pos = [];
    l = [];
    
    while(1)
        j = 1;
        for i=1:sv_num
            % 选出当前时刻的 某个卫星的所有观测
            PRN_obs.data = curr_obs.data(i,:);
            PRN_obs.col = curr_obs.col;
            PRN = PRN_obs.data(obs.col.PRN);
            C1 = PRN_obs.data(obs.col.C1);
            
            % 获得与当前时刻最接近的eph
            one_sv_eph = all_eph([all_eph(:,1) == PRN], :); % 读取某一个卫星的所有星历
            if isempty(one_sv_eph)
                continue; %当前卫星没有eph, pass
            end
            
            [~,idx] = min(abs(one_sv_eph(:, 17) - this_TOW)); % 挑这一颗卫星里找与当前时间最接近的那一套星历
            one_sv_eph = one_sv_eph(idx, :);
            
            M0 = one_sv_eph(2);
            Delta_n = one_sv_eph(3);
            e = one_sv_eph(4);
            sqrtA = one_sv_eph(5);
            OMEGA = one_sv_eph(6);
            i0 = one_sv_eph(7);
            omega =  one_sv_eph(8);
            OMEGA_DOT = one_sv_eph(9);
            iDOT = one_sv_eph(10);
            Cuc = one_sv_eph(11);
            Cus = one_sv_eph(12);
            Crc = one_sv_eph(13);
            Crs = one_sv_eph(14);
            Cic = one_sv_eph(15);
            Cis = one_sv_eph(16);
            toe = one_sv_eph(17);
            toc = one_sv_eph(20);
            a0 = one_sv_eph( 21);
            a1 = one_sv_eph(22);
            a2 = one_sv_eph(23);
            
            % 信号传播时间
            tau = C1./cv;
            
            %计算卫星钟差(包含相对论效应改正)
            sv_dt = sv_clock_bias(this_TOW , toc, a0, a1, a2, e, sqrtA, toe, Delta_n, M0);
            
            % 计算卫星位置
            [Xs, Ys, Zs, ~] = ch_sat_pos(this_TOW - tau, toc, a0, a1, a2, Crs, Delta_n, M0, Cuc, e, Cus, sqrtA, toe, Cic, OMEGA, Cis, i0, Crc, omega, OMEGA_DOT, iDOT);
            
            % 卫星位置地球自转校正
            spos = ch_sv_pos_rotate([Xs ;Ys; Zs], tau);
  
            %7、计算对流层延迟 dtrop
            dx = spos - X(1:3);
            [~, E1, ~] = topocent(X(1:3), dx);
            if isnan(geodeticHstation)
                [~,~,h] = togeod(a, finv, X(1), X(2), X(3));
                geodeticHstation=h*10^(-3);
            end
            if chooseTropo==1
                dtrop = tropo(sind(E1),geodeticHstation,P0,T,e0,geodeticHstation,geodeticHstation,geodeticHstation);
            elseif chooseTropo==2
                dtrop = tropo_error_correction(E1,geodeticHstation);
            end
            % dtrop=0;%暂不考虑dtrop
            %8、计算电离层延迟 diono
            diono = Error_Ionospheric_Klobuchar(X(1:3,1)',[Xs; Ys; Zs]', Alpha, Beta, this_TOW);
            
            l(j) = C1 + cv*sv_dt - dtrop - diono;
            sv_pos(j,:) = spos;
            j = j+1;
        end
        
        L = l;
        
        [X, residual, G] = ch_gpsls(X,  sv_pos',  L);

        %如果收敛到远离天际的一端，则重新迭代
        if (abs(norm(X(1:3)) - a) > 1000*100)
            X = [0 0 0 0]';
            break;
        else if norm(residual(1:3)) < 0.1
                rec_xyz(ii,:) = X;
                break;
            end
        end
    end
    
end

%% 删除定位失败的行
rec_xyz(all(rec_xyz==0,2),:) = [];

%% 验证
GT = [-2267749.30600679, 5009154.2824012134, 3221290.677045021]';
error = rec_xyz(:,1:3) - GT';

fprintf("偏差:%f std:%f\r\n", mean(vecnorm(error, 2, 2)), std(vecnorm(error, 2, 2)));

[lat, lon, h] = ch_ECEF2LLA(GT);

for i = 1: length(rec_xyz)
    [E, N, U] = ch_ECEF2ENU(rec_xyz(i, (1:3)), lat, lon, h);
    ENU(i,:) = [E, N, U];
end

figure;
subplot(3, 1, 1);
plot(ENU(:,1));
ylabel("E");
subplot(3, 1, 2);
plot(ENU(:,2));
ylabel("N");
subplot(3, 1, 3);
plot(ENU(:,3));
ylabel("U");



figure;
plot(ENU(:,1), ENU(:,2), '*');
xlabel("E"); ylabel("N");


figure;
plot(rec_xyz(:,4), '.-');
title("接收机钟差");

%
% %% plot
% plot(outdata.poslla(:,2), outdata.poslla(:,1), '.');
% xlabel('lon');
% ylabel('lat');
%
% [lat0, lon0, h0] = ch_ECEF2LLA(outdata.pos_ecef(1,:));
% for i = 1:length(outdata.pos_ecef)-1
%     [E, N, U]  = ch_ECEF2ENU(outdata.pos_ecef(i,:), lat0, lon0, h0 );
%     pos_enu(i,:) = [E; N; U];
% end
%
% pos_enu(end,:)
%
%
% %  [az, el] = satellite_az_el(outdata.pos_sv(2,:)' , outdata.pos_ecef(1,:)');
% %  rad2deg(az)
% %  rad2deg(el)
%
%
% figure;
% plot(pos_enu(:,1), pos_enu(:,2), '.');
% xlabel('E');
% ylabel('N');
%
% figure;
% subplot(3,1,1);
% plot(pos_enu(:,1));
% ylabel('E');
% subplot(3,1,2);
% plot(pos_enu(:,2));
% ylabel('N');
% subplot(3,1,3);
% plot(pos_enu(:,3));
% ylabel('U');
%
%
% figure;
% subplot(3,1,1);
% plot(outdata.HDOP);
% title('HDOP');
% subplot(3,1,2);
% plot(outdata.VDOP);
% title('VDOP');
% subplot(3,1,3);
% plot(outdata.usr_clk_bias/c);
% title('usc_clk_bias');
%
%
%
