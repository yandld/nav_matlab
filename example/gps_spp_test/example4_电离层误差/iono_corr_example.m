clear;
clc;
close all;


Alpha = [0.2235D-07  0.2235D-07 -0.1192D-06 -0.1192D-06];  %导航电文头文件中的电离层Alpha
Beta = [0.1290D+06  0.4915D+05 -0.1966D+06  0.3277D+06];   %导航电文头文件中的电离层Beta

RP = [ -2175464.65976786          4387261.15978363          4072912.71678943]';
SP = [  15987741.24878          3203328.88432069          21122447.2672636]';

this_TOW = 225119;

   diono = iono_correction(RP ,SP, Alpha, Beta, this_TOW);
   
   [lat,  lon, h] = ch_ECEF2LLA(RP);
   [az, el] = satellite_az_el(SP, RP);
   
     delay = iono_error_correction(rad2deg(lat), rad2deg(lon), rad2deg(az), rad2deg(el), this_TOW, [Alpha, Beta]);
   

     
   delay
diono

      %%     3.07222426505461e-08
      