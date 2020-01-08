%% Start of script
addpath('../../library')
addpath('../../library/nav_lib')
close all;
clear;
clc;

%% Import and plot sensor data
load('ExampleData.mat');
%%  plot sensor data 
imu_data_plot('acc', Accelerometer, 'gyr', Gyroscope, 'mag', Magnetometer, 'time', time);
T = mean(diff(time)); % sample time 

%% Process sensor data through algorithm

Gyroscope = Gyroscope * (pi/180);

q1 = zeros(length(time), 4);

q2 = q1;
q3 = q1;

q1(1,:) = [1 0 0 0];
q2(1,:) = [1 0 0 0];
q3(1,:) = [1 0 0 0];

%% intgate
for i = 2:length(time)
	q1(i, :) = mahony.imu(q1(i-1, :), Gyroscope(i, :), Accelerometer(i, :), T, 2);
	%q2(i, :) = fourati.imu(q2(i-1, :), Gyroscope(i, :), Accelerometer(i, :), T, 1, 0.3);
    %q1(i,:) = quatint.m(q1(i-1,:), Gyroscope(i, :), T);
	%q2(i,:) = quatint.q(q2(i-1,:), Gyroscope(i, :), T);
end


%% EKF A Double-Stage Kalman Filter for Orientation Tracking With an Integrated Processor in 9-D IMU

Q = 10e-6 *eye(4);
R = 2*eye(3);
P = ones(4)*0.125;
% P(1,1) = 0.001;
% P(2,2) = 0.001;
% P(3,3) = 0.001;
% P(4,4) = 0.001;

for i = 2:length(time)
    gyr = Gyroscope(i,:);
    
    fx = @(X, F) raw_fx(X, gyr, T );

    %quaternion2(i, :) = (F * quaternion2(i-1, :)')';
	[q2(i, :), P] = ekf(q2(i-1, :)', Accelerometer(i,:)', R, Q, P, fx, @hx);
end



%% Plot algorithm output as Euler angles
eul_plot('q1', q1, 'q2', q2, 'time', time);

% state transition
function [X, Jacob] = raw_fx(X, gyr, T)

    F = [0 -gyr(1) -gyr(2) -gyr(3); gyr(1) 0 gyr(3) -gyr(2); gyr(2) -gyr(3) 0 gyr(1); gyr(3) gyr(2) -gyr(1) 0];
    F = 0.5* T * F + eye(4);
    
    X = F * X;
    
    X = X / norm(X);
    
    Jacob = F;
end

% function for measurement
function [Val, Jacob] = hx(X)

q0 = X(1);
q1 = X(2);
q2 = X(3);
q3 = X(4);

Val(1) = 2*q1*q3 - 2*q0*q2;
Val(2) =   2*q0*q1 + 2*q2*q3;
Val(3) =  q0^2 - q1^2 - q2^2 + q3^2;
Val = Val';

Jacob(1,:) = [ -2*q2,  2*q3, -2*q0, 2*q1];
Jacob(2,:) = [  2*q1,  2*q0,  2*q3, 2*q2];
Jacob(3,:) = [  2*q0, -2*q1, -2*q2, 2*q3];
end


%% End of script