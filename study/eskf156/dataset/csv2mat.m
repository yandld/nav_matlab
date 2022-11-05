close all;
clear;
clc;

file_name = '2022年11月05日22时46分58秒';


data = csvread(strcat(file_name, '.csv'), 1);
data_length = length(data);

time = data(:,2);
plot(diff(time));

save(file_name, 'data');

% 
% 
% close all;
% clear;
% clc;
% 
% file_list = ls("*.csv");
% 
% for i=1:size(file_list,1)
%     file_name = strtrim(file_list(i,:));
%     disp(file_name);
% 
%     data = csvread(file_name, 1);
%     
%     save(file_name(1:end-4), 'data');
% end
