close all;
clear;
clc;

file_list = ls("*.csv");

for i=1:size(file_list,1)
    file_name = strtrim(file_list(i,:));
    disp(file_name);

    data = csvread(file_name, 1);
    
    save(file_name(1:end-4), 'data');
end