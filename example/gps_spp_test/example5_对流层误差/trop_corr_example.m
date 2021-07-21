clear;
clc;
close all;




RP = [ -2175464.65976786          4387261.15978363          4072912.71678943]';
SP = [  15987741.24878          3203328.88432069          21122447.2672636]';

this_TOW = 225119;



[lat,  lon, h] = ch_ECEF2LLA(RP);
[az, el] = satellite_az_el(SP, RP);

el = linspace(0.1, pi*0.9, 100);

 dtrop = tropo_correction(el, 1*1000);

plot(dtrop)
title("对流层误差");
ylabel("m");


