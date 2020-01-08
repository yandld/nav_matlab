function g = ch_gravity_ned(L_b,h_b)
%Gravity_ECEF - Calculates  acceleration due to gravity resolved about 
%north, east, and down
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 2/4/2012 by Paul Groves
%
% Inputs:
%   L_b           latitude (rad)
%   h_b           height (m)
% Outputs:
%   g       Acceleration due to gravity (m/s^2)

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Parameters
R_0 = 6378137; %WGS84 Equatorial radius in meters
R_P = 6356752.31425; %WGS84 Polar radius in meters
e = 0.0818191908425; %WGS84 eccentricity
f = 1 / 298.257223563; %WGS84 flattening
mu = 3.986004418E14; %WGS84 Earth gravitational constant (m^3 s^-2)
omega_ie = 7.292115E-5;  % Earth rotation rate (rad/s)

% Begins

% Calculate surface gravity using the Somigliana model, (2.134)
sinsqL = sin(L_b)^2;
g_0 = 9.7803253359 * (1 + 0.001931853 * sinsqL) / sqrt(1 - e^2 * sinsqL);

% Calculate north gravity using (2.140)
g(1,1) = -8.08E-9 * h_b * sin(2 * L_b);

% East gravity is zero
g(2,1) = 0;

% Calculate down gravity using (2.139)
g(3,1) = g_0 * (1 - (2 / R_0) * (1 + f * (1 - 2 * sinsqL) +...
    (omega_ie^2 * R_0^2 * R_P / mu)) * h_b + (3 * h_b^2 / R_0^2));

% Ends