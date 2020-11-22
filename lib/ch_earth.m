
function [R_meridian, R_transverse, Cecef2ned, Cecef2enu, gravity]= ch_earth(lat, lon, hgt)
% R_meridian： 南北向地球曲率半径
% R_transverse：东西向地球曲率半径

R0 = 6378137;               %WGS84 Equatorial radius in meters
e = 0.0818191908425;    %WGS84 eccentricity
% Calculate meridian radius of curvature using (2.105)
temp = 1 - (e * sin(lat))^2;
R_meridian = R0 * (1 - e^2) / temp^1.5;

% Calculate transverse radius of curvature using (2.105)
R_transverse = R0 / sqrt(temp);

clat = cos(lat);
slat = sin(lat);
clon = cos(lon);
slon = sin(lon);

Cecef2enu(1,:) =  [-slon ,             clon,                 0];
Cecef2enu(2,:) = [ -slat*clon,    -slat*slon          clat];
Cecef2enu(3,:) = [ clat*clon,      clat*slon,          slat];
           

Cecef2ned(1,:) = [-slat*clon,     -slat * slon,       clat];
Cecef2ned(2,:) = [-slon,             clon,                    0];
Cecef2ned(3,:) = [ -clat*clon,   -clat*slon,        -slat];

% Calculate surface gravity using the Somigliana model, (2.134)
sinsqL = slat^2;
gravity = 9.7803253359 * (1 + 0.001931853 * sinsqL) / sqrt(1 - e^2 * sinsqL);

end