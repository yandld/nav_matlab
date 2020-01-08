%% Start of script

addpath('../../library'); 
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% simulate data 
% create the test data:
% radii
a = 1;
b = 2;
c = 3;
[ s, t ] = meshgrid( 0 : 1 : 2*pi, 0 : 1 :2* pi );
x = a * cos(s) .* cos( t );
y = b * cos(s) .* sin( t );
z = c * sin(s);
% rotation
ang = pi/8;
ang = 0;

xt = x * cos( ang ) - y * sin( ang );
yt = x * sin( ang ) + y * cos( ang );
% translation
shiftx = 1;
shifty = 2;
shiftz = 3;
x = xt + shiftx;
y = yt + shifty;
z = z  + shiftz;

% add noise:
noiseIntensity = 0.21; %
dx = randn( size( s ) ) * noiseIntensity;
dy = randn( size( s ) ) * noiseIntensity;
dz = randn( size( s ) ) * noiseIntensity;
x = x + dx;
y = y + dy;
z = z + dz;
x = x(:);
y = y(:);
z = z(:);
fprintf( 'Simulated average data deviation: %.5f\n', sqrt( sum( dx(:).^2 + dy(:).^2 + dz(:).^2 ) / size( x, 1 ) ) );


% %% data from HSI
% DATA =[
%     0.0713,    2.0561,   -2.9719;
%    -0.3838,    2.0882,   -2.6589;
%    -0.7347,    2.0669,   -1.8178;
%    -0.9690,    2.0190,   -0.6218;
%    -0.9067,    2.0369,    0.7551;
%    -0.7776,    2.0461,    1.9223;
%    -0.4267,    2.0982,    2.7973;
%     0.0522,    2.0156,    3.0638;
%     0.0097,    2.0856,   -2.9042;
%    -0.1887,    1.3860,   -2.6788;
%    -0.4057,    0.8151,   -1.8029;
%    -0.5356,    0.4946,   -0.6387;
%    -0.5929,    0.5184,    0.7347;
%    -0.4215,    0.8257,    1.9400;
%    -0.2187,    1.3336,    2.7097;
%     0.0973,    2.0590,    3.0255;
%     0.0649,    2.0226,   -2.9776;
%     0.1766,    1.1925,   -2.6361;
%     0.2194,    0.5338,   -1.7860;
%     0.2602,    0.1242,   -0.6331;
%     0.2995,    0.1281,    0.7456;
%     0.1823,    0.5373,    1.9380;
%     0.1099,    1.1805,    2.7036;
%     0.0173,    2.0824,    3.0602;
%     0.0391,    2.0983,   -2.9613;
%     0.4741,    1.6965,   -2.6113;
%     0.7847,    1.3559,   -1.8704;
%     0.8844,    1.2124,   -0.6213;
%     0.9183,    1.1648,    0.7100;
%     0.7571,    1.4122,    1.9166;
%     0.4326,    1.7115,    2.7799;
%     0.0657,    2.0818,    3.0322;
%     0.0628,    2.0261,   -2.9215;
%     0.4201,    2.4359,   -2.6558;
%     0.7476,    2.6807,   -1.8669;
%     0.8799,    2.8885,   -0.6500;
% ];
% 
% 
% %% generate data
%    x = DATA(:,1);
%    y = DATA(:,2);
%    z = DATA(:,3);

%% fit_test 1 
[center, radii, rotM, evecs, v1, chi2]=ellipsoid_fit([x y z], 1);

fprintf( 'Ellipsoid center: %.5g %.5g %.5g\n', center );
fprintf( 'Ellipsoid radii: %.5g %.5g %.5g\n', radii );
fprintf( 'Ellipsoid evecs:\n' );
fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form:\n' );
fprintf( '%.5g ', v1 );
fprintf( '\nAverage deviation of the fit: %.5f\n', sqrt( chi2 / size( x, 1 ) ) );

cal_res_3dplot_show([x y z], v1);

fprintf('\r\n');
%% fit_test 2
[ center, radii, evecs, v2, chi2 ] = ellipsoid_fit_new( [ x y z ], '0' );

fprintf( 'Ellipsoid center: %.5g %.5g %.5g\n', center );
fprintf( 'Ellipsoid radii: %.5g %.5g %.5g\n', radii );
fprintf( 'Ellipsoid evecs:\n' );
fprintf( '%.5g %.5g %.5g\n%.5g %.5g %.5g\n%.5g %.5g %.5g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form:\n' );
fprintf( '%.5g ', v2 );
fprintf( '\nAverage deviation of the fit: %.5f\n', sqrt( chi2 / size( x, 1 ) ) );
fprintf( '\n' );

 cal_res_3dplot_show([x y z], [v1; v2']);
%% End Of Scripts