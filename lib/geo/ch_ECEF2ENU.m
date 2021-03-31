function [E, N, U] = ch_ECEF2ENU(XYZ, lat0, lon0, h0)

% ECEF坐标转ENU
% lat0, lon0, h0: 起始点经纬高, 经纬度为rad， 高度为m
% XYZ ECEF坐标  单位为m
% ENU距离 单位为m
[lat, lon, h] = ch_ECEF2LLA(XYZ);

[E, N, U] =  ch_LLA2ENU(lat, lon, h,  lat0, lon0, h0);

end
