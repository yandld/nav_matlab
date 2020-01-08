%%
% A new calibration method for tri-axial field sensors in strap-down navigation systems   其实就是点积不变法

%% Start of script
addpath('../library', '../library/mag_cal');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                 % clear the command terminal


%% real data 
path = '../data_set/UranusData.csv';

[a ,g, m, EularAngle, time] = imu_csv_import(path);
a = a / 1000;
g = g / 10;
m = m / 10;
 
[acc, gyr, mag] = imu_data_purify(a, g, m, 0.01, 60);

 gB = acc;
 mB = mag;
 


%% plot  data
figure;
plot3(gB(:,1), gB(:,2), gB(:,3), '.r');
title('Gravity in body frame');

figure;
plot3(mB(:,1), mB(:,2), mB(:,3), '.b');
title('Magnetometer in body frame');

%% feature normalize
feature_scale = 50;
mB = mB ./ feature_scale;


%% DPI
%  [mis, bias] = dpi(gB, mB, 1* h * cos(deg2rad(90 - 58)));
%  
%  fprintf('using DPI mehold:\n');
%  if(mis(1,1) < 0)
%      mis = -mis;
%      bias = -bias;
%  end
%  
% mis ./ mis(1,1)
% bias .* feature_scale

%% DIP5 model

x0(1:3) = 0; % bx by bz
x0(4) = 1;  %   norm_h 
x0(5) = 1;  %   lambada
% X: [Bx By Bz norm_h lambada]
 [~, bias, norm_h, lamda, inter, J] = dip5 (gB, mB, x0, 0.001);

 fprintf('using DIP5 mehold:\n');
 
 figure;
 plot(1:length(J), J, '*-');
 title('DIP5');
 
fprintf('inter = %d\n', inter);

bias = bias' .* feature_scale;
bias
fprintf('incli: %f\r\n',  rad2deg(acos(lamda)) - 90);
fprintf('norm_h %f\r\n', norm_h*feature_scale);

%% DIP13 model
 [mis, bias, lamda, inter, J] = dip13 (gB, mB, norm_h, 0.001);

 fprintf('using DIP13 mehold:\n');
 
 figure;
 plot(1:length(J), J, '*-');
  title('DIP13');
  
fprintf('inter = %d\n', inter);

%mis = mis ./ mis(1,1);
mis

bias' .* feature_scale
fprintf('incli: %f\r\n',  rad2deg(acos(lamda)) - 90);




