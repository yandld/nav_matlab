%%
clc;
clear;
close all;
addpath('../library'); 

%% load data 
sample_rate = 200;

X =  csvread('../data_set/20180627.csv',1, 1);
Accelerometer = X(:,2:4);
Gyroscope = X(:,5:7);
Gyroscope = Gyroscope / 100;
Magnetometer = X(:,8:10);
time = 1:length(Accelerometer);
time = time / sample_rate;
EulerRaw = X(:,11:13);

%%  plot sensor data 
imu_data_plot(Accelerometer, Gyroscope, Magnetometer, EulerRaw, time);

%% cal
Cs = [1.0041   -0.0165    0.0050; 0.0149    1.0089    0.0105; -0.0032   -0.0027    1.0024];
bias = [0.1854   -0.1719   -0.0396]';

for t = 1:length(time)
    Gyroscope_Cal(t,:)=Cs*(Gyroscope(t,:)-bias')';
end


%% integate
Gyroscope = Gyroscope * (pi/180);
Gyroscope_Cal = Gyroscope_Cal * (pi/180);

quaternion1 = zeros(length(time), 4);
quaternion2 = zeros(length(time), 4);

quaternion1(1,:) = [1 0 0 0];
quaternion2(1,:) = [1 0 0 0];

for t = 2:length(time)
        quaternion1(t,:) = quatint.bk_single(quaternion1(t-1,:), Gyroscope(t,:), 1 / sample_rate);
        quaternion2(t,:) = quatint.bk_single(quaternion2(t-1,:), Gyroscope_Cal(t,:), 1 / sample_rate);
end

%% Plot
figure('Name', 'Euler Angles');
hold on;
euler = quat2eul((quaternion1), 'ZYX') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
plot(time, euler(:,3), 'r.');
plot(time, euler(:,2), 'g.');
plot(time, euler(:,1), 'b.');

euler = quat2eul((quaternion2), 'ZYX') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
plot(time, euler(:,3), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,1), 'b');


title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');


legend('raw\phi', 'raw\theta', 'raw\psi','cal\phi', 'cal\theta', 'cal\psi');
hold off;

%% End of Script
