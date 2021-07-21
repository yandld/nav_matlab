function [az, el] = satellite_az_el(SP, RP)
%% 计算卫星俯仰角(el), 和方位角(az)
% get_satellite_az_el: computes the satellite azimuth and elevation
% given the position of the user and the satellite in ECEF
% : SP:    卫星位置ECEF
%  RP:    用户位置ECEF
% Output Args:
% az: azimuth: 方位角
% el: elevation： 俯仰角
% 参考 GPS 原理及接收机设计 谢刚

[lat, lon, ~] = ch_ECEF2LLA(RP);

C_ECEF2ENU = [-sin(lon) cos(lon) 0; -sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat); cos(lat)*cos(lon) cos(lat)*sin(lon) sin(lat)];

enu = C_ECEF2ENU*[SP - RP];

% nroamlized line of silght vector
enu = enu / norm(enu);

az = atan2(enu(1), enu(2));
el = asin(enu(3)/norm(enu));
end


