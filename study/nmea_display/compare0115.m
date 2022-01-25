close all;
clear;
clc;

%%
[by_lat, by_lon, by_alt, by_vel, by_att, pos_std, vel_std, att_std, ins_status, pos_type] = inspvaxa2pvax('./0115/BY.txt');
[ag3335_lat, ag3335_lon, ag3335_alt, ag3335_status] = gga2pos('./0115/AG.txt');
[rac_lat, rac_lon, rac_alt, rac_status] = gga2pos('./0115/RAC.txt');

%%
wm = webmap('World Imagery');
wmline(ag3335_lat, ag3335_lon, 'Color', 'green', 'Width', 1, 'OverlayName', 'AG3335');
wmline(rac_lat, rac_lon, 'Color', 'cyan', 'Width', 1, 'OverlayName', 'RAC');
% wmline(by_lat, by_lon, 'Color', 'magenta', 'Width', 1, 'OverlayName', 'RAC');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'
