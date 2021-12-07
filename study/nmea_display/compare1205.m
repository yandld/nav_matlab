close all;
clear;
clc;

[f9p_lat, f9p_lon, f9p_alt, f9p_status] = nmea2pos('F9P.ubx');
[ag3335_lat, ag3335_lon, ag3335_alt, ag3335_status] = nmea2pos('AG3335.txt');
[m8p1_lat, m8p1_lon, m8p1_alt, m8p1_status] = nmea2pos('M8P_1.txt');
[m8p2_lat, m8p2_lon, m8p2_alt, m8p2_status] = nmea2pos('M8P_2.txt');
[rac_lat, rac_lon, rac_alt, rac_status] = nmea2pos('RAC.txt');

[f9p_neu] = pos2xyz(f9p_lat, f9p_lon, f9p_alt);
[ag3335_neu] = pos2xyz(ag3335_lat, ag3335_lon, ag3335_alt);
[m8p1_neu] = pos2xyz(m8p1_lat, m8p1_lon, m8p1_alt);
[m8p2_neu] = pos2xyz(m8p2_lat, m8p2_lon, m8p2_alt);
[rac_neu] = pos2xyz(rac_lat, rac_lon, rac_alt);

%%
wm = webmap('World Imagery');
wmline(f9p_lat, f9p_lon, 'Color', 'blue', 'Width', 1, 'OverlayName', 'F9P');
wmline(ag3335_lat, ag3335_lon, 'Color', 'green', 'Width', 1, 'OverlayName', 'AG3335');
wmline(m8p1_lat, m8p1_lon, 'Color', 'red', 'Width', 1, 'OverlayName', 'M8P-1');
wmline(m8p2_lat, m8p2_lon, 'Color', 'magenta', 'Width', 1, 'OverlayName', 'M8P-2');
wmline(rac_lat, rac_lon, 'Color', 'white', 'Width', 1, 'OverlayName', 'RAC');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'