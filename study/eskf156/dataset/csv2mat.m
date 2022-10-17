close all;
clear;
clc;

file_name = '2022年10月12日16时38分35秒';

data = csvread(strcat(file_name, '.csv'), 1);
data_length = length(data);

plot(diff(data(:,2)));

save(file_name, 'data');