function gravity = ch_gravity(lat, h)

% gravity: 计算重力(暂时不考虑高度)
%
% INPUT
%       lat,纬度(rad)
%       h,  高度(m), 暂时没有用
gravity = 9.780325 * ( 1 + 0.0053024*sin(lat)^(2) - 0.0000058*sin(lat*2)^(2));


