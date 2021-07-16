% EASY9   Various representations of an estimated baseline

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/07/27  $

% Master and Rover Positions Given From Postprocessing of GPS Observations
pos1 = [3436412.0323; 603295.9341; 5321480.4962];
pos2 = [3435470.7981; 607792.3243; 5321592.3777];
fprintf('\nX1 %12.4f [m], Y1 %12.4f [m], Z1 %12.4f [m]',pos1(1),pos1(2),pos1(3))
fprintf('\nX2 %12.4f [m], Y2 %12.4f [m], Z2 %12.4f [m]',pos2(1),pos2(2),pos2(3))

% Baseline in Cartesian Coordinates
fprintf('\n\nBaseline Represented in Cartesian Coordinates\n')
fprintf('\ndelta X: %7.3f [m], delta Y:  %7.3f [m], delta Z:  %7.3f [m]', ...
    pos2(1)-pos1(1),pos2(2)-pos1(2),pos2(3)-pos1(3))
length = norm(pos2-pos1);
fprintf('\n\nLength of Baseline %12.3f [m]',length)

% Covariance Matrix Given From Postprocessing of GPS Observations
fprintf('\n\nVariance of the Components')
Sigma_XYZ = [5^2 -.797*5*2 .911*5*4;
    -.797*5*2 2^2 -.795*2*4;
    .911*5*4 -.795*2*4 4^2];
std = sqrt(diag(Sigma_XYZ));          
fprintf('\n\nsigma_X: %4.1f [mm], sigma_Y: %4.1f [mm], sigma_Z: %4.1f [mm]\n', ...
    std(1),std(2),std(3))

% Conversion of (X,Y,Z) to (phi, lambda, height)
[phi1,lambda1,h1] = togeod(6378137,298.257223563,pos1(1,1),pos1(2,1),pos1(3,1));
phi1_dms = deg2dms(phi1);
fprintf('\nPhi1  %12.0f %5.0f %5.5f',phi1_dms(1),phi1_dms(2),phi1_dms(3))
lambda1_dms = deg2dms(lambda1);
fprintf('\nLambda1 %7.0f %5.0f %5.5f',lambda1_dms(1),lambda1_dms(2),lambda1_dms(3))
fprintf('\nHeight1 %12.4f [m]\n',h1)

[phi2,lambda2,h2] = togeod(6378137,298.257223563,pos2(1,1),pos2(2,1),pos2(3,1));
phi2_dms = deg2dms(phi2);
fprintf('\nPhi2  %12.0f %5.0f %5.5f',phi2_dms(1),phi2_dms(2),phi2_dms(3))
lambda2_dms = deg2dms(lambda2);
fprintf('\nLambda2 %7.0f %5.0f %5.5f',lambda2_dms(1),lambda2_dms(2),lambda2_dms(3))
fprintf('\nHeight2 %12.4f [m]\n',h2)

% Variance transformation of baseline components from (X,Y,Z) to 
% (phi,lambda,height) or equivalently (E,N,U)
phi = phi1*pi/180;
lambda = lambda1*pi/180;
F = [ -sin(lambda) -sin(phi)*cos(lambda) cos(phi)*cos(lambda);
       cos(lambda) -sin(phi)*sin(lambda) cos(phi)*sin(lambda);
                0               cos(phi)             sin(phi)]; % [e n u]
Sigma = [  25     -7.97 18.22;
           -7.97   4    -6.36;
           18.22  -6.36 16   ];
F_inv = inv(F);  

% Complete covariance matrix for UTM coordinates
Sigma_ENU = F_inv*Sigma*F_inv'; 
std = sqrt(diag(Sigma_ENU));
fprintf('\n\nsigma_N: %4.1f [mm] sigma_E: %4.1f [mm] sigma_U: %4.1f [mm]', ...
    std(2),std(1),std(3))

% Conversion of (X,Y,Z) to topocentric coordinates (Az, El, Dist)
fprintf(['\n\nConversion of (delta X, delta Y, delta Z) to ', ...
        'Topocentric Coordinates (Azimuth, Elevation, Distance):'])
[az1,el1,d1] = topocent(pos1,pos2-pos1);
fprintf('\n\nAz1: %12.5f [deg] El1:  %6.4f [deg] D1:  %9.4f [m]',az1, el1, d1)
[az2,el2,d2] = topocent(pos2,pos1-pos2);
fprintf('\nAz2: %12.5f [deg] El2:  %6.4f [deg]  D2:  %9.4f [m]',az2, el2, d2)
u1 = h2-h1;
fprintf('\nUp1 %12.4f',u1)
u2 = h1-h2;
fprintf('\nUp2 %12.4f',u2)

% Computation based on the geodesic between the terminals 
[s12, a1, a2] = bessel_2(phi1_dms(1),phi1_dms(2),phi1_dms(3),lambda1_dms(1), ...
                 lambda1_dms(2),...
        lambda1_dms(3),phi2_dms(1),phi2_dms(2),phi2_dms(3),lambda2_dms(1),...
        lambda2_dms(2),lambda2_dms(3),6378137,298.257223563);
fprintf('\n\nComputation of Geodesic on the WGS Ellipsoid\n')
fprintf('\nLength of Geodesic %12.4f [m]',s12)
A1 = rad2dms0(a1);
fprintf('\nAzimuth from 1 to 2:  %6.0f  [deg] %4.0f  [min] %8.4f [sec]',A1(1), A1(2),A1(3))
A2 = rad2dms0(a2);
fprintf('\nAzimuth from 2 to 1:  %6.0f  [deg] %4.0f  [min] %8.4f [sec]\n',A2(1), A2(2),A2(3))

% Conversion of Geocentric Coordinates (X,Y,Z) to (N,E,U) in UTM
zone = round(30+(lambda*180/pi+3)/6) %;
[N1,E1,U1] = wgs2utm(pos1(1),pos1(2),pos1(3),zone) %;
[N2,E2,U2] = wgs2utm(pos2(1),pos2(2),pos2(3),zone) %;
fprintf('\n\n')
E = E2-E1
N = N2-N1
U = U2-U1
% Transformation of local spherical (Az,El,D) to local Cartesian Coordinates (x,y,h)
fprintf('\nRepresentation of Topocentric System as East, North, and Up')
%az1 = az1-((zone-30)*6-3)*pi/180 %%%%%%
x1 = d1*cos(el1*pi/180)*sin(az1*pi/180);
y1 = d1*cos(el1*pi/180)*cos(az1*pi/180);
u1 = h2-h1;
fprintf('\n\nEast1 %12.4f',x1)
fprintf('\nNorth1 %12.4f',y1)
fprintf('\nUp1 %12.4f',u1)
x2 = d2*cos(el2*pi/180)*sin(az2*pi/180);
y2 = d2*cos(el2*pi/180)*cos(az2*pi/180);
u2 = h1-h2;
fprintf('\n\nEast2 %12.4f',x2)
fprintf('\nNorth2 %12.4f',y2)
fprintf('\nUp2 %12.4f\n',u2)

%%%%%%%%%%%%%%%%%%%%%% end easy9.m  %%%%%%%%%%%%%%%%%%%

