clear;
clc;
close all;

%% 磁倾角计算，跟具体的坐标系没有关系

%% ENU frame
Gn = [ 0 0 -1]; %重力在ENU下的表示,
Mn = [-3.49 28.076 -46.995]; %北京地区ENU系下地磁场

%归一化
Gn = Gn / norm(Gn);
Mn  = Mn / norm(Mn);

inclination_angle = pi/2 - acos(dot(Gn,Mn));
inclination_angle = rad2deg(inclination_angle);

declination = rad2deg(atan2(Mn(1), Mn(2)));

fprintf('ENU系下 磁倾角 %f°\n',inclination_angle);
fprintf('ENU系下 磁偏角 %f°\n',declination);

%% NED frame
Gn = [ 0 0 1]; 
Mn = [28.076, -3.49  46.995]; 

Gn = Gn / norm(Gn);
Mn  = Mn / norm(Mn);


inclination_angle = pi/2 - acos(dot(Gn,Mn));
inclination_angle = rad2deg(inclination_angle);

declination = rad2deg(atan2(Mn(2), Mn(1)));

fprintf('NED系下 磁倾角 %f°\n',inclination_angle);
fprintf('NED系下 磁偏角 %f°\n',declination);

%可以通过 https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm 验证


fprintf('结论: 磁倾角和磁偏角和具体的坐标系选择无关\n');
