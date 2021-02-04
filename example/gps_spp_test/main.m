clear;
clc;
close all;

%%

% 星历 参考GPS原理及接收机设计 3.4
eph.rcvr_tow = 100;
eph.sqrtA = 5153.65531;
eph.toe = 244800;
eph.dn = 4.249105564E-9;
eph.m0 = -1.064739758;
eph.e = 0.005912038265;
eph.w = -1.717457876;
eph.i0 = 0.9848407943;
eph.idot = 7.422851197E-51;
eph.omg0 = 1.038062244;
eph.odot = -8.151768125E-9;
eph.cus = 2.237036824E-6;
eph.cuc = 3.054738045E-7;
eph.crs = 2.53125;
eph.crc = 350.53125;
eph.cis = 8.940696716E-8;
eph.cic = -8.381903172E-8;

%  当前GPS时间
current_gpst = 239050.7223;

sv_pos = get_satellite_position(eph, current_gpst, 1);
sv_pos


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
outdata.pos = [];
outdata.HDOP =[];
outdata.VDOP = [];
outdata.usr_clk_bias = [];

% initial position of the user
pos = [0 0 0]';

% initial clock bias
bias = 0;

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
    b = 0;
    xu = [ 0 0 0]';
    
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
            pr_raw = sv_data(2);
            pr_(end+1) = pr_raw + c*dsv;
        end
        % Now lets calculate the satellite positions and construct the G
        % matrix. Then we'll run the least squares optimization to
        % calculate corrected user position and clock bias. We'll iterate
        % until change in user position and clock bias is less than a
        % threhold. In practice, the optimization converges very quickly,
        % usually in 2-3 iterations even when the starting point for the
        % user position and clock bias is far away from the true values.
        dx = 100*ones(1,3); db = 100;
        while(norm(dx) > 0.1 && norm(db) > 1)
            %	while(norm(dx) > 0.1 && norm(db) > 1)
            Xs = []; % concatenated satellite positions
            pr = []; % pseudoranges corrected for user clock bias
            
            % 计算每一颗卫星的位置
            for i = 1: sv_num
                cpr = pr_(i) - b;
                pr = [pr; cpr];
                
                % Signal transmission time
                tau = cpr/c;
                
                sv_pos = get_satellite_position(eph_formatted_{i}, rcvr_tow-tau, 1);
                sv_pos = sv_pos_rotate(sv_pos, tau);
                Xs = [Xs; sv_pos'];
            end
            
            % 最小二乘解算
            [usr_ecef, bias, ~, G] = ch_gpsls(xu, b, Xs',  pr');
            	dx = usr_ecef - xu;
				db = bias - b;
				xu = usr_ecef;
				b = bias;
        end
        

            
        % ECEF转 LLA
        [lat, lon, h] = ch_ecef2lla(usr_ecef);
        
        % 计算DOP
        [VDOP, HDOP, ~, ~] = ch_gpsdop(G, lat, lon);
        
        % 保存数据
        outdata.HDOP(end+1,:) = HDOP;
        outdata.VDOP(end+1,:) = VDOP;
        outdata.pos(end+1,:) = [rad2deg(lat) rad2deg(lon) h];
        outdata.usr_clk_bias(end+1,:) = bias;
        
    end
end

%% plot
plot(outdata.pos(:,2), outdata.pos(:,1), '.');
xlabel('lon');
ylabel('lat');

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



%% 根据星历计算卫星位置
function pos = get_satellite_position(eph, t, compute_harmonic_correction)
% get_satellite_position: computes the position of a satellite at time (t) given the
% ephemeris parameters.
% Usage: [x y z] =  get_satellite_position(eph, t, compute_harmonic_correction)
% Input Args: eph: ephemeris data
%             t: time
%             compute_harmonic_correction (optional): 1 if harmonic
%             correction should be applied, 0 if not.
% Output Args: [x y z] in ECEF in meters
% ephmeris data must have the following fields:
% rcvr_tow (receiver tow)
% svid (satellite id)
% toc (reference time of clock parameters)
% toe (referece time of ephemeris parameters)
% af0, af1, af2: clock correction coefficients
% ura (user range accuracy)
% e (eccentricity)
% sqrtA (sqrt of semi-major axis)
% dn (mean motion correction)
% m0 (mean anomaly at reference time)
% w (argument of perigee)
% omg0 (lontitude of ascending node)
% i0 (inclination angle at reference time)
% odot (rate of right ascension)
% idot (rate of inclination angle)
% cus (argument of latitude correction, sine)
% cuc (argument of latitude correction, cosine)
% cis (inclination correction, sine)
% cic (inclination correction, cosine)
% crs (radius correction, sine)
% crc (radius correction, cosine)
% iod (issue of data number)


% set default value for harmonic correction
switch nargin
    case 2
        compute_harmonic_correction=1;
end
mu = 3.986005e14;
omega_dot_earth = 7.2921151467e-5; %(rad/sec)

% Now follow table 20-IV
A = eph.sqrtA^2;
cmm = sqrt(mu/A^3); % computed mean motion
tk = t - eph.toe;
% account for beginning of end of week crossover
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
% 迭代3次
Ek = mk;
Ek = mk + eph.e*sin(Ek);
Ek = mk + eph.e*sin(Ek);
Ek = mk + eph.e*sin(Ek);

% syms E;
% eqn = E - eph.e*sin(E) == mk;
% solx = vpasolve(eqn, E);
% Ek = double(solx);

% True anomaly:
nu = atan2((sqrt(1-eph.e^2))*sin(Ek)/(1-eph.e*cos(Ek)), (cos(Ek)-eph.e)/(1-eph.e*cos(Ek)));
%Ek = acos((e  + cos(nu))/(1+e*cos(nu)));

%升交点角距
Phi = nu + eph.w;
du = 0;
dr = 0;
di = 0;

% 摄动校正量
if (compute_harmonic_correction == 1)
    % compute harmonic corrections
    du = eph.cus*sin(2*Phi) + eph.cuc*cos(2*Phi);
    dr = eph.crs*sin(2*Phi) + eph.crc*cos(2*Phi);
    di = eph.cis*sin(2*Phi) + eph.cic*cos(2*Phi);
end

u = Phi + du;
r = A*(1-eph.e*cos(Ek)) + dr;

% inclination angle at reference time
i = eph.i0 + eph.idot*tk + di;
x_prime = r*cos(u);
y_prime = r*sin(u);
omega = eph.omg0 + (eph.odot - omega_dot_earth)*tk - omega_dot_earth*eph.toe;

pos = [0 0 0]';

pos(1) = x_prime*cos(omega) - y_prime*cos(i)*sin(omega);
pos(2) = x_prime*sin(omega) + y_prime*cos(i)*cos(omega);
pos(3) = y_prime*sin(i);

end

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

%% 计算经过地球自转改正后的卫星位置
function sv_pos = sv_pos_rotate(sv_pos, tau)

% Earth's rotation rate
omega_e = 7.2921151467e-5; %(rad/sec)
theta = omega_e * tau;
sv_pos = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1]*sv_pos;
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

% syms E;
% eqn = E - eph.e*sin(E) == mk;
% solx = vpasolve(eqn, E);
% Ek = double(solx);

% dsv 时间为s
dsv = eph.af0 + eph.af1*(t-eph.toc) + eph.af2*(t-eph.toc)^2 + F*eph.e*eph.sqrtA*sin(Ek);

end




% function [x, b, norm_dp, G] = estimate_position(xs, pr, numSat, x0, b0, dim)
	% estimate_position: estimate the user's position and user clock bias
	% Usage: [x, b, norm_dp, G] = estimate_position(xs, pr, numSat, x0, b0, dim)
	% Input Args: xs: satellite position matrix
	%             pr: corrected pseudo ranges (adjusted for known value of the
	%             satellite clock bias)
	%             numSat: number of satellites
	%             x0: starting estimate of the user position
	%             b0: starting point for the user clock bias
	%             dim: dimensions of the satellite vector. 3 for 3D, 2 for 2D
	% Notes: b and b0 are usually 0 as the current estimate of the clock bias
	% has already been applied to the input pseudo ranges.
	% Output Args: x: optimized user position
	%              b: optimized user clock bias
	%              norm_dp: normalized pseudo-range difference
	%              G: user satellite geometry matrix, useful for computing DOPs
%  
% 	dx = 100*ones(1, dim);
% 	db = 0;
% 	norm_dp = 100;
% 	b = b0;
% 	%while (norm_dp > 1e-4)
% 	while norm(dx) > 1
% 		norms = sqrt(sum((xs-x0).^2,2));
% 		% delta pseudo range:
% 		dp = pr - norms + b - b0;
% 		G = [-(xs-x0)./norms ones(numSat,1)];
% 		sol = inv(G'*G)*G'*dp;
% 		dx = sol(1:dim)';
% 		db = sol(dim+1);
% 		norm_dp = norm(dp);
% 		x0 = x0 + dx;
% 		b0 = b0 + db;
% 	end
% 	x = x0;
% 	b = b0;
% end
    