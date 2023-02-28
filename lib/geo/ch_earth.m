
function [Rns, Rew, C_ECEF2ENU, C_ECEF2NED]= ch_earth(lat, lon, h)

%% 根据经纬度计算地球常用参数
% INPUT
% lat: 纬度(rad)
% lon: 经度(rad)

% OUTPUT
% Rns, R_meridian(RM, ns)： 南北向地球曲率半径, 子午圈曲率半径(竖着的)
% Rew_transverse(RN, ew)：东西向地球曲率半径, 卯酉圈曲率半径(横着的)
% C_ECEF2ENU: ECEF到ENU转换矩阵
% C_ECEF2NED: ECEF到NED转换矩阵


R0 = 6378137;               %WGS84 赤道半径
e = 0.0818191908425;    %WGS84 eccentricity
% Calculate meridian radius of curvature using (2.105)
temp = 1 - (e * sin(lat))^2;
Rns = R0 * (1 - e^2) / temp^1.5;

% Calculate transverse radius of curvature using (2.105)
Rew = R0 / sqrt(temp);

clat = cos(lat);
slat = sin(lat);
clon = cos(lon);
slon = sin(lon);

C_ECEF2ENU(1,:) =  [-slon ,             clon,                 0];
C_ECEF2ENU(2,:) = [ -slat*clon,    -slat*slon          clat];
C_ECEF2ENU(3,:) = [ clat*clon,      clat*slon,          slat];
           

C_ECEF2NED(1,:) = [-slat*clon,     -slat * slon,       clat];
C_ECEF2NED(2,:) = [-slon,             clon,                    0];
C_ECEF2NED(3,:) = [ -clat*clon,   -clat*slon,        -slat];


end