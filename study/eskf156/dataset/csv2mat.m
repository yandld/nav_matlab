close all;
clear;
clc;

file_name = '2022年11月22日16时45分57秒';


data = csvread(strcat(file_name, '.csv'), 1);
N = length(data);

time = data(:,2);
diff_time = diff(time);
imu_data = data(:, 21:26);
gyr = imu_data(:, 1:3);
acc= imu_data(:, 4:6);

% fprintf("清洗数据...\r\n");
% acc(:,1) = filloutliers(acc(:,1), 'linear');
% acc(:,2) = filloutliers(acc(:,2), 'linear');
% acc(:,3) = filloutliers(acc(:,3), 'linear');
% data(:,24:26) = acc;

plot(diff_time);


fprintf("保存数据...\r\n");
save(fullfile(file_name + ".mat"), 'data');

%  file_list = ls("*.csv");
 
% 
% for i=1:size(file_list,1)
%     file_name = strtrim(file_list(i,:));
%     disp(file_name);
% 
%     data = csvread(file_name, 1);
%     
%     save(file_name(1:end-4), 'data');
% end
fprintf("保存完成\r\n");
