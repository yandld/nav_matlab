%% Start of script
close all;                         % close all figures
clear;                              % clear all variables
clc;                                 % clear the command terminal
addpath('../library');      % include quaternion library

%%
path = '../sample/HI216S¸ßµÍÎÂ²âÊÔÊý¾Ý.csv';
[Accelerometer, Gyroscope, Magnetometer, EularAngle, time] = imu_csv_import(path);

Gyroscope(:,1) = Gyroscope(:,1)  / 100;
Gyroscope(:,2) = Gyroscope(:,2)  / 100;
Gyroscope(:,3) = Gyroscope(:,3)  / 100;


Gyroscope(:,1)= filloutliers(Gyroscope(:,1),'previous','mean','ThresholdFactor',3);
Gyroscope(:,1) = Gyroscope(:,1) * 4;
Gyroscope(:,2)= filloutliers(Gyroscope(:,2),'previous','mean','ThresholdFactor',5);
Gyroscope(:,3)= filloutliers(Gyroscope(:,3),'previous','mean','ThresholdFactor',4);


imu_data_plot(Accelerometer, Gyroscope, Magnetometer, EularAngle, time);

