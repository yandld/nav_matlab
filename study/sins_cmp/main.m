close all;
clear;
clc;

format compact
format long g

rad = pi/180;
deg = 180/pi;

f = 400;
dt = 1/f;
alignment_time = 1*f;

data = readmatrix('UranusData2.csv');
data_length = length(data);
gyro_data = data(:, 6:8);

gyro_bias = mean(gyro_data(1:alignment_time, :));

fprintf('数据长度：%f秒\n',data_length*dt);

%% 100Hz
nQb_100Hz = [1 0 0 0];
data_interval = 4;
for i=1:data_interval:data_length
    w_b = (gyro_data(i,:) - gyro_bias)' * rad;
    nQb_100Hz = sins1(nQb_100Hz, w_b, dt*data_interval);
end
att_100Hz = q2att(nQb_100Hz)'*deg

%% 200Hz
nQb_200Hz = [1 0 0 0];
data_interval = 2;
for i=1:data_interval:data_length
    w_b = (gyro_data(i,:) - gyro_bias)' * rad;
    nQb_200Hz = sins1(nQb_200Hz, w_b, dt*data_interval);
end
att_200Hz = q2att(nQb_200Hz)'*deg

%% 400Hz
nQb_400Hz = [1 0 0 0];
data_interval = 1;
for i=1:data_interval:data_length
    w_b = (gyro_data(i,:) - gyro_bias)' * rad;
    nQb_400Hz = sins1(nQb_400Hz, w_b, dt*data_interval);
end
att_400Hz = q2att(nQb_400Hz)'*deg

%% 双子样
nQb2 = [1 0 0 0];
for i=2:2:data_length
    w_b = (gyro_data(i-1:i,:) - gyro_bias) * rad;
    nQb2 = sins2(nQb2, w_b, dt);
end
att_2sample = q2att(nQb2)'*deg

%% 四子样
nQb4 = [1 0 0 0];
for i=4:4:data_length
    w_b = (gyro_data(i-3:i,:) - gyro_bias) * rad;
    nQb4 = sins4(nQb4, w_b, dt);
end
att_4sample = q2att(nQb4)'*deg