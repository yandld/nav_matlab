close all;
clear;
clc;

file_name = '2022年05月27日14时59分34秒';

data = csvread(strcat(file_name, '.csv'), 1);
data_length = length(data);

plot(diff(data(:,2)));

save(file_name, 'data');