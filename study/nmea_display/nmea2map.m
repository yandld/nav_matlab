close all;
clear;
clc;

% [lat, lon, alt, time, status] = gga2pos('posnmea.txt');

% [lat, lon, alt, time, status] = gga2pos('./1128/RAC1.txt');
% [lat, lon, alt, time, status] = gga2pos('./1128/RAC2.txt');

% [lat, lon, alt, time, status] = gga2pos('./1205/F9P.ubx');

% [lat, lon, alt, time, status] = gga2pos('./0115/AG.txt');
% [lat, lon, alt, time, status] = gga2pos('./0115/RAC.txt');

[lat, lon, alt, time, status] = gga2pos('./0122/HI600RTK_GNGGA.txt');

[neu] = pos2xyz(lat, lon, alt);

%%
sg_xyz = neu(find(status==1), :);
dg_xyz = neu(find(status==2), :);
fixed_xyz = neu(find(status==4), :);
float_xyz = neu(find(status==5), :);

figure;
hold on; grid on;
legend_str = string([]);
if ~isempty(sg_xyz)
    plot(sg_xyz(:, 1), sg_xyz(:, 2), 'r.', 'LineWidth', 3); 
    legend_str = [legend_str; 'Standalone'];
end
if ~isempty(dg_xyz)
    plot(dg_xyz(:, 1), dg_xyz(:, 2), 'm.', 'LineWidth', 3);
    legend_str = [legend_str; 'DGNSS'];
end
if ~isempty(float_xyz)
    plot(float_xyz(:, 1), float_xyz(:, 2), 'b.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Float'];
end
if ~isempty(fixed_xyz)
    plot(fixed_xyz(:, 1), fixed_xyz(:, 2), 'g.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Fixed'];
end
plot(neu(:, 1), neu(:, 2), 'k'); 
axis equal;
xlabel('东向距离(m)');
ylabel('北向距离(m)');
legend(legend_str);

%%
figure;
hold on; grid on;
legend_str = string([]);
if ~isempty(sg_xyz)
    plot(find(status==1), sg_xyz(:, 3), 'r.', 'LineWidth', 3); 
    legend_str = [legend_str; 'Standalone'];
end
if ~isempty(dg_xyz)
    plot(find(status==2), dg_xyz(:, 3), 'm.', 'LineWidth', 3);
    legend_str = [legend_str; 'DGNSS'];
end
if ~isempty(float_xyz)
    plot(find(status==5), float_xyz(:, 3), 'b.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Float'];
end
if ~isempty(fixed_xyz)
    plot(find(status==4), fixed_xyz(:, 3), 'g.', 'LineWidth', 3);
    legend_str = [legend_str; 'RTK Fixed'];
end
xlabel('时间(s)');
ylabel('高度(m)');
xlim([0 length(neu)]);
legend(legend_str, 'Orientation', 'horizontal');

%%
wm = webmap('World Imagery');
wmline(lat, lon, 'Color', 'blue', 'Width', 2, 'OverlayName', 'GNSS');
% 线性可选颜色： 
% 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'
