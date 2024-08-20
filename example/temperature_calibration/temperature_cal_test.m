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

% 定义平滑度阈值和目标值
smoothness_threshold = 0.5;  % 可以根据需要调整
target_value = -100;
window_size = 200;

% 遍历每个IMU模块
for imu = 1:num_imus
    imu_folder = fullfile(main_dir, imu_folders(imu).name);
    
    % 遍历每个温度点
    for t = 1:length(temp_points)
        temp = temp_points(t);
        filename = fullfile(imu_folder, sprintf('Z_720_%d.csv', temp));
        
        % 读取CSV文件
        try
            data = readtable(filename);
            
            % 检查GyrZ数据
            gyrZ = data.GyrZ;
            
            % 计算移动标准差
            moving_std = movstd(gyrZ, window_size);
            
            % 找到平滑且接近目标值的区域
            valid_indices = find(abs(gyrZ - target_value) < 5 & moving_std < smoothness_threshold);
            
            if ~isempty(valid_indices)
                % 选择符合条件的第一个点作为起始点
                start_index = valid_indices(1);
                
                % 确保有足够的数据点
                if start_index + window_size - 1 <= length(gyrZ)
                    selected_data = data(start_index:start_index+window_size-1, :);
                    
                    for col = columns
                        all_data.(col{1})(imu, t) = mean(selected_data.(col{1}), 'omitnan');
                    end
                    fprintf('成功处理文件: %s\n', filename);
                else
                    fprintf('文件 %s 中没有足够的数据点\n', filename);
                end
            else
                fprintf('文件 %s 中没有找到符合条件的数据\n', filename);
            end
        catch
            fprintf('无法读取文件: %s\n', filename);
        end
    end
end

% 绘制每个轴的图表
figure();
for i = 1:length(columns)
    col = columns{i};
    subplot(2, 3, i);
    hold on;
    for imu = 1:num_imus
        plot(temp_points, all_data.(col)(imu, :), '.-', 'DisplayName', sprintf('%s', imu_folders(imu).name));
    end
    title(sprintf('%s 零偏随温度变化', col));
    xlabel('温度 (°C)');
    ylabel('零偏');
    legend('Location', 'best');
    grid on;
    hold off;
end

% 设置图形标题
sgtitle('IMU零偏随温度变化总览');
