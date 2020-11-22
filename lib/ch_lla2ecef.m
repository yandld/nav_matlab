function [xyz] = ch_lla2ecef(lat, lon, ght)

% Parameters
R0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Begins

% Calculate transverse radius of curvature using (2.105)
R_transverse = R0 / sqrt(1 - (e * sin(lat))^2);

% Convert position using (2.112)
clat = cos(lat);
slat = sin(lat);
clon = cos(lon);
slon = sin(lon);

xyz = [0 0 0]';

xyz(1) = (R_transverse + ght) * clat * clon;
xyz(2) = (R_transverse + ght) * clat * slon;
xyz(3) = ((1 - e^2) * R_transverse + ght) * slat;

      