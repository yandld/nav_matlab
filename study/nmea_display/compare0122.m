close all;
clear;
clc;

%%
[by_lat, by_lon, by_alt, by_vel, by_att, pos_std, vel_std, att_std, ins_status, pos_type, by_time] = inspvaxa2pvax('./0122/BY_INSPVAX.txt');
[hi600_lat, hi600_lon, hi600_alt, hi600_time, hi600_status] = gga2pos('./0122/HI600RTK_GNGGA.txt');
[rac_lat, rac_lon, rac_alt, rac_time, rac_status] = gga2pos('./0122/RAC_GPGGA.txt');

%%
wm = webmap('World Imagery');
wmline(hi600_lat, hi600_lon, 'Color', 'green', 'Width', 1, 'OverlayName', 'HI600');
wmline(rac_lat, rac_lon, 'Color', 'cyan', 'Width', 1, 'OverlayName', 'RAC');
wmline(by_lat, by_lon, 'Color', 'magenta', 'Width', 1, 'OverlayName', 'BY');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'

%%
rad = pi/180;
deg = 180/pi;
Re = 6378137;
Earth_e = 0.00335281066474748;

lat0 = by_lat(1)*rad;
h0 = 50;

Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
Rn = Re * (1 + Earth_e*sin(lat0)*sin(lat0));
Rmh = Rm + h0;
Rnh = Rn + h0;

lat_error = rac_lat - by_lat;
lon_error = rac_lon - by_lon;

lat_error = lat_error * rad * (Rmh);
lon_error = lon_error * rad * (Rnh) * cos(lat0);
pos_error = sqrt(lat_error.^2+lon_error.^2);

% figure;
% plot(pos_error, 'LineWidth', 2); grid on;
% xlim([0 length(pos_error)]);
% xlabel('时间(s)');
% ylabel('水平位置误差(m)');
% title('水平位置误差');

sg_e = pos_error(find(hi600_status==1));
dg_e = pos_error(find(hi600_status==2));
fix_e = pos_error(find(hi600_status==4));
float_e = pos_error(find(hi600_status==5));

figure;
hold on; grid on;
legend_str = string([]);
if ~isempty(sg_e)
    plot(find(hi600_status==1), sg_e, 'r.', 'LineWidth', 3); 
    legend_str = [legend_str; 'Standalone'];
end
if ~isempty(dg_e)
    plot(find(hi600_status==2), dg_e, 'm.', 'LineWidth', 3);
    legend_str = [legend_str; 'DGNSS'];
end
if ~isempty(float_e)
    plot(find(hi600_status==5), float_e, 'b.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Float'];
end
if ~isempty(fix_e)
    plot(find(hi600_status==4), fix_e, 'g.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Fixed'];
end
xlim([0 length(pos_error)]);
xlabel('时间(s)');
ylabel('水平位置误差(m)');
title('RAC 水平位置误差');
legend(legend_str, 'Orientation', 'horizontal');

%%
figure;
subplot(2,1,1);
plot(by_alt, 'LineWidth', 1,  'Marker', '.'); hold on; grid on;
plot(hi600_alt+9.3154, 'LineWidth', 1, 'Marker', '.');
plot(rac_alt, 'LineWidth', 1, 'Marker', '.');
legend('北云', 'HI600-RTK', 'RAC', 'Orientation', 'horizontal');
xlim([0 length(pos_error)]);
xlabel('时间(s)');
ylabel('高度(m)');
title('高度及高度误差');
subplot(2,1,2);
alt_error = hi600_alt + 9.3154 - by_alt;
sg_e = alt_error(find(hi600_status==1));
dg_e = alt_error(find(hi600_status==2));
fix_e = alt_error(find(hi600_status==4));
float_e = alt_error(find(hi600_status==5));
hold on; grid on;
legend_str = string([]);
if ~isempty(sg_e)
    plot(find(hi600_status==1), sg_e, 'r.', 'LineWidth', 3); 
    legend_str = [legend_str; 'Standalone'];
end
if ~isempty(dg_e)
    plot(find(hi600_status==2), dg_e, 'm.', 'LineWidth', 3);
    legend_str = [legend_str; 'DGNSS'];
end
if ~isempty(float_e)
    plot(find(hi600_status==5), float_e, 'b.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Float'];
end
if ~isempty(fix_e)
    plot(find(hi600_status==4), fix_e, 'g.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Fixed'];
end
xlim([0 length(pos_error)]);
xlabel('时间(s)');
ylabel('高度误差(m)');
legend(legend_str, 'Orientation', 'horizontal');
