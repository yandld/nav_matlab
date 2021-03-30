clear;
clc;
close all;
format short

latd = [45 45 45 0 90 60];

for i = 1:6
 gravity = ch_gravity(deg2rad(latd(i)), 0);
 fprintf("纬度:%f  -> 重力:%f\n", latd(i), gravity);
end 