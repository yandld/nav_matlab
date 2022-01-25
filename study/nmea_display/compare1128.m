close all;
clear;
clc;

%%
[mtk_lat, mtk_lon, mtk_alt, mtk_status] = nmea2pos('./1128/MTK1.TXT');
[rac_lat, rac_lon, rac_alt, rac_status] = nmea2pos('./1128/RAC1.TXT');

% [mtk_lat, mtk_lon, mtk_alt, mtk_status] = nmea2pos('./1128/MTK2.TXT');
% [rac_lat, rac_lon, rac_alt, rac_status] = nmea2pos('./1128/RAC2.TXT');

%%
wm = webmap('World Imagery');
wmline(mtk_lat, mtk_lon, 'Color', 'blue', 'Width', 1, 'OverlayName', 'MTK');
wmline(rac_lat, rac_lon, 'Color', 'green', 'Width', 1, 'OverlayName', 'RAC');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'