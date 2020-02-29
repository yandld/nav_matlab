function [R_meridian, R_transverse]= ch_earth(lat, hgt)

R0 = 6378137;               %WGS84 Equatorial radius in meters
e = 0.0818191908425;    %WGS84 eccentricity
% Calculate meridian radius of curvature using (2.105)
temp = 1 - (e * sin(lat))^2;
R_meridian = R0 * (1 - e^2) / temp^1.5;

% Calculate transverse radius of curvature using (2.105)
R_transverse = R0 / sqrt(temp);
end