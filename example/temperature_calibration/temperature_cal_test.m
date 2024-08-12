close all;
clc;
clear;


% 设置主目录
main_dir = '20240808_ch010_temperature';

% 获取所有IMU文件夹
imu_folders = dir(fullfile(main_dir, '0*'));
num_imus = length(imu_folders);

% 定义温度点
temp_points = 0:10:120;

% 定义要提取的列
columns = {'AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ'};

% 初始化一个结构体来存储所有数据
all_data = struct();
for col = columns
    all_data.(col{1}) = zeros(num_imus, length(temp_points));
end

% 遍历每个IMU模块
for imu = 1:num_imus
    imu_folder = fullfile(main_dir, imu_folders(imu).name);
    
    % 遍历每个温度点
    for t = 1:length(temp_points)
        temp = temp_points(t);
        filename = fullfile(imu_folder, sprintf('still_30s_%d.csv', temp));
        
        % 读取CSV文件
        try
            data = readtable(filename);
            for col = columns
                all_data.(col{1})(imu, t) = mean(data.(col{1}), 'omitnan');
            end
            fprintf('成功读取文件: %s\n', filename);
        catch
            fprintf('无法读取文件: %s\n', filename);
            % 如果文件不存在或无法读取，保持为0
        end
    end
end

% 设置颜色映射
colors = lines(num_imus);

% 绘制每个轴的图表
figure();
for i = 1:length(columns)
    col = columns{i};
    subplot(2, 3, i);
    hold on;
    for imu = 1:num_imus
        plot(temp_points, all_data.(col)(imu, :), '.-', 'Color', colors(imu,:), ...
             'DisplayName', sprintf('%s', imu_folders(imu).name));
    end
    title(sprintf('%s 零偏随温度变化', col));
    xlabel('温度 (°C)');
    ylabel('零偏');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % 设置坐标轴范围，确保所有点都可见
    axis_range = axis;
    axis([min(temp_points) max(temp_points) axis_range(3:4)]);
end

% 调整子图之间的间距
set(gcf, 'Name', 'IMU零偏随温度变化');
sgtitle('IMU零偏随温度变化总览');

