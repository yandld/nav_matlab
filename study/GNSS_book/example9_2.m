clear;
clc
close all;
addpath('../../library/nav_lib');


%% DOP 

%% Satellite information
theta = deg2rad([0 0 0 -90]); % 仰角
alpha = deg2rad([0 120 240 0]); %方位角

theta = deg2rad([-30 -30 -30 -60]); %Elevation
alpha = deg2rad([60 90 120 90]);% Azimuth

%% calculate Geometry matrix
%% 注意这里使用NED坐标系！！
H = [-cos(theta').*cos(alpha)'   -cos(theta').*sin(alpha)'  -sin(theta') ones(length(theta), 1)];
G = (H'*H)^(-1);

NDOP = sqrt(G(1,1));
EDOP = sqrt(G(2,2));
VDOP = sqrt(G(3,3));
TDOP = sqrt(G(4,4));
HDOP = sqrt(G(1,1) + G(2,2));
PDOP = sqrt (G(1,1) + G(2,2) + G(3,3));
GDOP = sqrt(trace(G));


