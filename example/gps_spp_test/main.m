clear;
clc;
close all;



%% begin main
% Constants that we will need
% Speed of light
c = 299792458;


% load out data
data = load('rtcm_data.mat');

% Data is a cell array containing data about different RTCM messages. We
% are interested in 1002 and 1019
% msgs is a an array of message ids (1002, 1019 etc)
msgs = [data.data{1,:}];

% Get indicies of ephemeris info (msg 1019)
[idx_1019] = find(msgs == 1019);
% Get indicies of raw pseudorange info (msg 1002)
[idx_1002] = find(msgs == 1002);
% Satellite ephemeris data is all mixed up since different satellites are visible at
% different epocks.
eph = [data.data{2, idx_1019}];
% Lets group data for each satellite
% find unique satellite indicies
sv_arr = unique(eph(1,:));
% eph_data will contain ephemeris data for all epochs grouped by satellite
% number
eph_data = {};

for i = 1: length(sv_arr)
    % find indicies of all entries corresponding to this satellite
    sv = sv_arr(i);
    idx = find(eph(1,:) == sv);
    eph_data{sv} = eph(:,idx);
end
% Now let's deal with 1002 messages. 1002 messages have two entries - first
% one (nav1) is a 6*1 array containing: reference station id, receiver time
% of week, number of satellites etc. See decode_1002.m in goGPS for details
% The important pieces of info in nav1 are the receiver time of week and the
% number of satellites visible
% The second one (nav2) contains a block of 56*5 for every epoch.
% Num of rows (56) refers to the maximum number of satellites in the
% constellation. Num of cols (5) is the number of data elements for each satellite.
% We are interested in the second element, the raw pseudorange.
% For those satellites for which no info is available,
% the rows of nav2 contain 0s.
nav1 = [data.data{2, idx_1002}];
nav2 = [data.data{3, idx_1002}];
len = length(nav1);

% 存储输出数据
outdata.poslla = [];
outdata.pos_ecef = [];
outdata.HDOP =[];
outdata.VDOP = [];
outdata.usr_clk_bias = [];



% 1002 messages are spaced apart 200ms. Let's use 1 out of every 5 samples.
% This means that we'll compute position every second, which is sufficient
for idx = 1: 5: len
    % second element of nav1 contains receiver time of week
    rcvr_tow = nav1(2, idx);
    
    % data block corresponding to this satellite
    nav_data = nav2(:, 5*(idx-1)+1: 5*idx);
    % find indicies of rows containing non-zero data. Each row corresponds
    % to a satellite
    ind = find(sum(nav_data,2) ~= 0);
    sv_num = length(ind);
    eph_formatted_ = [];
    
    % The minimum number of satellites needed is 4, let's go for more than that to be more robust
    % initial position of the user ,  initial clock bias
    delta = [100 100 100 100]';
    X = [ 0 0 0 0 ]';
    
    
    
    if (sv_num > 4)
        pr_ = [];
        % Correct for satellite clock bias and find the best ephemeris data
        % for each satellite. Note that satellite ephemeris data (1019) is sent
        % far less frequently than pseudorange info (1002). So for every
        % epoch, we find the closest (in time) ephemeris data.
        for i = 1: sv_num
            sv_idx = ind(i);
            sv_data = nav_data(sv_idx,:);
            % find ephemeris data closest to this time of week
            [c_, eph_idx] = min(abs(eph_data{sv_idx}(18,:)-rcvr_tow));
            eph_ = eph_data{sv_idx}(:, eph_idx);
            % Convert the ephemeris data into a standard format so it can
            % be input to routines that process it to calculate satellite
            % position and satellite clock bias
            eph_formatted = format_ephemeris3(eph_);
            eph_formatted_{end+1} = eph_formatted;
            % To be correct, the satellite clock bias should be calculated
            % at rcvr_tow - tau, however it doesn't make much difference to
            % do it at rcvr_tow
            dsv = estimate_satellite_clock_bias(rcvr_tow, eph_formatted);
            
            % measured pseudoranges corrected for satellite clock bias.
            % Also apply ionospheric and tropospheric corrections if
            % available
            pr = sv_data(2);
            pr_(end+1) = pr + c*dsv;
        end
        % Now lets calculate the satellite positions and construct the G
        % matrix. Then we'll run the least squares optimization to
        % calculate corrected user position and clock bias. We'll iterate
        % until change in user position and clock bias is less than a
        % threhold. In practice, the optimization converges very quickly,
        % usually in 2-3 iterations even when the starting point for the
        % user position and clock bias is far away from the true values.
        while(norm(delta) > 0.01)
            %	while(norm(dx) > 0.1 && norm(db) > 1)
            Xs = []; % concatenated satellite positions
            pr = []; % pseudoranges corrected for user clock bias
            
            % 计算每一颗卫星的位置
            for i = 1: sv_num
                cpr = pr_(i) - X(4);
                pr = [pr; cpr];
                
                % Signal transmission time
                tau = cpr/c;
                
                sqrtA = eph_formatted_{i}.sqrtA;
                toe = eph_formatted_{i}.toe;
                Delta_n = eph_formatted_{i}.dn;
                M0 = eph_formatted_{i}.m0;
                e =eph_formatted_{i}.e;
                omega = eph_formatted_{i}.w;
                i0 = eph_formatted_{i}.i0;
                iDOT = eph_formatted_{i}.idot;
                OMEGA = eph_formatted_{i}.omg0;
                OMEGA_DOT = eph_formatted_{i}.odot;
                Cus = eph_formatted_{i}.cus;
                Cuc =eph_formatted_{i}.cuc;
                Crs = eph_formatted_{i}.crs;
                Crc = eph_formatted_{i}.crc;
                Cis = eph_formatted_{i}.cis;
                Cic = eph_formatted_{i}.cic;
                
                %                 toc = eph_formatted_{i}.toc;
                %                 a0 = eph_formatted_{i}.af0;
                %                 a1 = eph_formatted_{i}.af1;
                %                 a2 = eph_formatted_{i}.af2;
                toc = 0;
                a0 = 0;
                a1 = 0;
                a2 = 0;
                
                [x_sat, y_sat, z_sat, Deltat] = ch_sat_pos(rcvr_tow-tau, toc, a0, a1, a2, Crs, Delta_n, M0, Cuc, e, Cus, sqrtA, toe, Cic, OMEGA, Cis, i0, Crc, omega, OMEGA_DOT, iDOT);
                sv_pos = [x_sat, y_sat, z_sat]';
                sv_pos = ch_sv_pos_rotate(sv_pos, tau);
                Xs = [Xs; sv_pos'];
                outdata.pos_sv(i,:) = sv_pos;
            end
            
            % 最小二乘解算
            [X, delta, G] = ch_gpsls(X, Xs',  pr');
        end
        
        % ECEF转 LLA
        [lat, lon, h] = ch_ECEF2LLA(X);
        
        % 计算DOP
        [VDOP, HDOP, ~, ~] = ch_gpsdop(G, lat, lon);
        
        % 保存数据
        outdata.pos_ecef(end+1, :) = X(1:3);
        outdata.HDOP(end+1,:) = HDOP;
        outdata.VDOP(end+1,:) = VDOP;
        outdata.poslla(end+1,:) = [rad2deg(lat) rad2deg(lon) h];
        outdata.usr_clk_bias(end+1,:) = X(4);
        
    end
end

%% plot
plot(outdata.poslla(:,2), outdata.poslla(:,1), '.');
xlabel('lon');
ylabel('lat');

[lat0, lon0, h0] = ch_ECEF2LLA(outdata.pos_ecef(1,:));
for i = 1:length(outdata.pos_ecef)-1
    [E, N, U]  = ch_ECEF2ENU(outdata.pos_ecef(i,:), lat0, lon0, h0 );
    pos_enu(i,:) = [E; N; U];
end

pos_enu(end,:)


%  [az, el] = satellite_az_el(outdata.pos_sv(2,:)' , outdata.pos_ecef(1,:)');
%  rad2deg(az)
%  rad2deg(el)


figure;
plot(pos_enu(:,1), pos_enu(:,2), '.');
xlabel('E');
ylabel('N');

figure;
subplot(3,1,1);
plot(pos_enu(:,1));
ylabel('E');
subplot(3,1,2);
plot(pos_enu(:,2));
ylabel('N');
subplot(3,1,3);
plot(pos_enu(:,3));
ylabel('U');


figure;
subplot(3,1,1);
plot(outdata.HDOP);
title('HDOP');
subplot(3,1,2);
plot(outdata.VDOP);
title('VDOP');
subplot(3,1,3);
plot(outdata.usr_clk_bias/c);
title('usc_clk_bias');




%% 读取星历
function eph = format_ephemeris3(eph_)
eph = [];
eph.svid = eph_(1);
eph.toc = eph_(21);
eph.toe = eph_(18);
eph.af0 = eph_(19);
eph.af1 = eph_(20);
eph.af2 = eph_(2);
eph.ura = eph_(26); % check
eph.e = eph_(6);
eph.sqrtA = eph_(4);
eph.dn = eph_(5);
eph.m0 = eph_(3);
eph.w = eph_(7);
eph.omg0 = eph_(16);
eph.i0 = eph_(12);
eph.odot = eph_(17);
%eph.wdot = eph_(17);
eph.idot = eph_(13);
eph.cus = eph_(9);
eph.cuc = eph_(8);
eph.cis = eph_(15);
eph.cic = eph_(14);
eph.crs = eph_(11);
eph.crc = eph_(10);
eph.iod = eph_(22);
eph.GPSWeek = eph_(24);
end




%% 计算卫星端钟差
function dsv = estimate_satellite_clock_bias(t, eph)
F = -4.442807633e-10;
mu = 3.986005e14;
A = eph.sqrtA^2;
cmm = sqrt(mu/A^3); % computed mean motion
tk = t - eph.toe;
% account for beginning or end of week crossover
if (tk > 302400)
    tk = tk-604800;
end
if (tk < -302400)
    tk = tk+604800;
end
% apply mean motion correction
n = cmm + eph.dn;

% Mean anomaly
mk = eph.m0 + n*tk;

% solve for eccentric anomaly
Ek = mk;
Ek = mk + eph.e*sin(Ek);
Ek = mk + eph.e*sin(Ek);
Ek = mk + eph.e*sin(Ek);

% dsv 时间为s
dsv = eph.af0 + eph.af1*(t-eph.toc) + eph.af2*(t-eph.toc)^2 + F*eph.e*eph.sqrtA*sin(Ek);

end

