function XYZ = ch_ENU2ECEF(E, N, U, lat0, lon0, h0)
% ENU  转 ECEF
% lat0, lon0, h0: 起始点经纬高, 经纬度为rad， 高度为m
% ENU东北天距离，单位为m
% 返回 ECEF坐标 单位m

[lat, lon, h] = ch_ENU2LLA(E, N, U, lat0, lon0, h0);
XYZ = ch_LLA2ECEF(lat, lon, h);

end
