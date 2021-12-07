close all;
clear;
clc;

data = importdata('./1205/F9P.ubx');

lat_save = [];
lon_save = [];
alt_save = [];
rtk_status = [];
for i = 1:length(data)
    str = char(data(i));
    if length(str)>6 && (strcmp(str(1:6),'$GNGGA') || strcmp(str(1:6),'$GPGGA'))
        sstr = string(str);
        sstr = sstr.split(',');
        
        lat = sstr(3);
        lat = lat.split('.');
        lat1 = char(lat(1));
        lat_deg = str2double(lat1(1:end-2));
        lat_min = str2double(lat1(end-1:end)) + str2double(strcat('0.',lat(2)));
        lat = lat_deg + lat_min/60;
        
        lat_save = [lat_save; lat];
        
        lon = sstr(5);
        lon = lon.split('.');
        lon1 = char(lon(1));
        lon_deg = str2double(lon1(1:end-2));
        lon_min = str2double(lon1(end-1:end)) + str2double(strcat('0.',lon(2)));
        lon = lon_deg + lon_min/60;
        
        lon_save = [lon_save; lon];
        
        alt = str2double(sstr(10));
        alt_save = [alt_save; alt];
        
        rtk = str2double(sstr(7));
        rtk_status = [rtk_status; rtk];
    end
end

%%
rad = pi/180;
lat_deg = 180/pi;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;
lat0 = lat_save(1)*rad;
h0 = alt_save(1);
Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
Rn = Re * (1 + Earth_e*sin(lat0)*sin(lat0));
Rmh = Rm + h0;
Rnh = Rn + h0;
gps_xyz = [];
for i = 1:length(lon_save)
    gps_xyz(i, 3) = alt_save(i) - alt_save(1);
    gps_xyz(i, 2) = (lat_save(i) - lat_save(1)) * rad * (Rmh);
    gps_xyz(i, 1) = (lon_save(i) - lon_save(1)) * rad * (Rnh) * cos(lat0);
end

% figure;
% comet(gps_xyz(:, 1), gps_xyz(:, 2), 'LineWidth', 3);
% grid on;
% axis equal;
% xlabel('东向距离(m)');
% ylabel('北向距离(m)');

% figure;
% plot(gps_xyz(:, 1), gps_xyz(:, 2), 'LineWidth', 3);
% grid on;
% axis equal;
% xlabel('东向距离(m)');
% ylabel('北向距离(m)');

% figure; plot(gps_xyz(:,3), 'LineWidth', 2); grid on;

%%
sg_xyz = gps_xyz(find(rtk_status==1), :);
dg_xyz = gps_xyz(find(rtk_status==2), :);
fix_xyz = gps_xyz(find(rtk_status==4), :);
float_xyz = gps_xyz(find(rtk_status==5), :);

figure;
hold on; grid on;
legend_str = string([]);
if ~isempty(sg_xyz)
    plot(sg_xyz(:, 1), sg_xyz(:, 2), '.', 'LineWidth', 3);
    legend_str = [legend_str; 'Standalone'];
end
if ~isempty(dg_xyz)
    plot(dg_xyz(:, 1), dg_xyz(:, 2), '.', 'LineWidth', 3);
    legend_str = [legend_str; 'DGNSS'];
end
if ~isempty(float_xyz)
    plot(float_xyz(:, 1), float_xyz(:, 2), '.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Float'];
end
if ~isempty(fix_xyz)
    plot(fix_xyz(:, 1), fix_xyz(:, 2), '.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Fix'];
end
axis equal;
xlabel('东向距离(m)');
ylabel('北向距离(m)');
legend(legend_str);

%%
wm = webmap('World Imagery');
wmline(lat_save, lon_save, 'Color', 'blue', 'Width', 3, 'OverlayName', 'GNSS');  hold on;
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'
