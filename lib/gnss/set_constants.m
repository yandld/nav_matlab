%*******************************************************
%
% DESCRIPTION:
% 		This script contains many useful constants for GPS
%       and related work.  It should be kept in only
%       one place so that updates are immediately available 
%       to all other scripts/functions.
%  
% ARGUMENTS:
% 		None, just call this script to place 
% 		the constants in your workspace.
%  
% OUTPUT:
% 		Variables in your current workspace.
%  
% CALLED BY:
% 		Many other codes.
%
% FUNCTIONS CALLED:
% 		None.
%
% MODIFICATIONS:    
%       XX-XX-02  :  Jan Weiss - Original
%       07-25-04  :  Jan Weiss - updated header.
%       10-19-04  :  Jan Weiss - Cleanup and added 
%                                some conversion factors.
%                 :  See SVN log for further updates.
% 
% Colorado Center for Astrodynamics Research
% Copyright 2005 University of Colorado, Boulder
%*******************************************************

% GENERAL CONSTANTS
% =========================================================================
c = 299792458;          %----> Speed of light (meters/s).
Re = 6378137 ;          %----> Earth Radius (meters)
% =========================================================================


% CONVERSION FACTORS
% =========================================================================
Hz2MHz = 1E-6;
MHz2Hz = 1E6;
s2ns = 1E9;
ns2s = 1E-9;
s2micros = 1E6;
micros2s = 1E-6;
s2ms = 1E3;
ms2s = 1E-3;
dtr = pi / 180;
rtd = 180 / pi;
m2cm = 100;
cm2m = 1 / 100;
m2mm = 1000;
mm2m = 1 / 1000;
ft2m = 0.3048;  % Source: http://www.nodc.noaa.gov/dsdt/ucg/
m2ft = 1 / 0.3048;
ns2m = c * ns2s;  % Converts time in nano-sec to distance,
                  % assuming the speed of light.
% =========================================================================
  

% GNSS SPECIFIC CONSTANTS
% =========================================================================
L1 = 1575.42e6;         %----> Freqs in Hz.
L2 = 1227.60e6;
L5 = 1176.45e6;

L1MHz = 1575.42;        %----> Freqs in MHz.
L2MHz = 1227.60;
L5MHz = 1176.45;

L1GHz = 1.57542;        %----> Freqs in GHz.
L2GHz = 1.22760;
L5GHz = 1.17645;

LAMBDA_L1 = c / L1;     %----> Wavelengths in meters.
LAMBDA_L2 = c / L2;
LAMBDA_L5 = c / L5;

CA_CODE_RATE = 1.023e6; %----> C/A and P code chipping rate in chips/s.
P_CODE_RATE = 10.23e6;

CA_CHIP_PERIOD = 1 / CA_CODE_RATE;   %----> C/A & P code chip periods in s.
P_CHIP_PERIOD = 1 / P_CODE_RATE;

CA_CHIP_LENGTH = c / CA_CODE_RATE;  %----> C/A & P code chip lengths in meters.
P_CHIP_LENGTH = c / P_CODE_RATE;

CA_CODE_LENGTH = 1023;  % chips
% =========================================================================

