function [lat, lon, h] = ch_ENU2LLA(E, N ,U, lat0, lon0, h0)

% ENU 转 经纬高
% E, N ,U 系下增量，单位为m
% lat0, lon0, h0: 起始点经纬高, 经纬度为rad， 高度为m
% lat, lon, h 终点经纬高, 经纬度为rad， 高度为m

%精确解
XYZ0 = ch_LLA2ECEF(lat0, lon0, h0);
[~, ~, C_ECEF2ENU, ~]= ch_earth(lat0, lon0, h0);
dXYZ = C_ECEF2ENU' * [E N U]';
XYZ = dXYZ + XYZ0;
[lat, lon, h] = ch_ECEF2LLA(XYZ);



% % 近似算法
% R_0 = 6378137; %WGS84 Equatorial radius in meters
% clat = cos(lat0);
%
% dlon = E / (R_0 * clat);
% dlat = N / R_0;
% dh = U;
%
% lon = lon0 + dlon;
% lat = lat0 + dlat;
% h = h0 + dh;


end