function [XYZ] = ch_LLA2ECEF(lat, lon, h)

% 经纬高转ECEF坐标
% lat:纬度(rad)
% lon:经度(rad)
% h高度(m)

R0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Calculate transverse radius of curvature using (2.105)
RN = R0 / sqrt(1 - (e * sin(lat))^2);

% Convert position using (2.112)
clat = cos(lat);
slat = sin(lat);
clon = cos(lon);
slon = sin(lon);

XYZ = [0 0 0]';

XYZ(1) = (RN + h) * clat * clon;
XYZ(2) = (RN + h) * clat * slon;
XYZ(3) = ((1 - e^2) * RN + h) * slat;

      