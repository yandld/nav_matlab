function [E, N, U] = ch_LLA2ENU(lat, lon, h, lat0, lon0, h0)

% 经纬高 转 ENU
% lat0, lon0, h0: 起始点经纬高, 经纬度为rad， 高度为m
% lat, lon, h 终点经纬高, 经纬度为rad， 高度为m
% E, N ,U 系下增量，单位为m

%精确算法
XYZ0 = ch_LLA2ECEF(lat0, lon0, h0);
XYZ1 = ch_LLA2ECEF(lat, lon, h);
dXYZ = XYZ1 - XYZ0;

 [~, ~, C_ECEF2ENU, ~]= ch_earth(lat0, lon0, h0);
 dENU = C_ECEF2ENU * dXYZ;
 E = dENU(1);
 N= dENU(2);
 U = dENU(3);
 
 %近似算法
% R_0 = 6378137; %WGS84 Equatorial radius in meters
% clat = cos(lat0);
% E = (lon - lon0) * clat * R_0;
% N =  (lat - lat0) * R_0;
% U = h - h0;
end
