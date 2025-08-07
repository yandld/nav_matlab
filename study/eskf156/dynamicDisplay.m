function dynamicDisplay(log, data, play_speed, start_time)
%DYNAMICDISPLAY 一行代码显示卡尔曼滤波动态结果
%   dynamicDisplay(log, data, play_speed, start_time) 显示所有滤波结果的动态播放
%
% 输入参数:
%   log        - 包含滤波结果的结构体，包含以下字段：
%                pos: [N×3] 位置 (东向, 北向, 天向) [m]
%                vel: [N×3] 速度 (东向, 北向, 天向) [m/s]
%                roll, pitch, yaw: [N×1] 姿态角 [rad]
%                installangle: [N×3] 安装角 (pitch, roll, yaw) [deg]
%                od_scale_factor: [N×1] 里程计比例因子
%                gyr_bias: [N×3] 陀螺零偏 [rad/s]
%                acc_bias: [N×3] 加速度计零偏 [m/s²]
%   data       - 包含原始数据的结构体，包含以下字段：
%                data.imu.tow: [N×1] 时间戳 [s]
%                data.imu.gyr: [N×3] 陀螺仪原始数据 [rad/s]
%                data.imu.acc: [N×3] 加速度计原始数据 [m/s²]
%   play_speed - 播放速度控制 (可选，默认为50)
%                数值越小播放越慢，数值越大播放越快
%                建议范围: 1-100
%   start_time - 开始播放时间 (可选，默认为0，从头开始)
%                单位：秒，基于data.imu.tow的时间戳
%                如果指定时间超出数据范围，将从最接近的时间开始
%
% 使用示例:
%   dynamicDisplay(log, data);                    % 默认参数：速度50，从0秒开始
%   dynamicDisplay(log, data, 30);                % 速度30，从0秒开始
%   dynamicDisplay(log, data, 50, 100);           % 速度50，从100秒开始播放
%   dynamicDisplay(log, data, [], 200);           % 默认速度，从200秒开始播放
%   dynamicDisplay(log, data, 20, 150);           % 速度20，从150秒开始播放
%
% 显示内容:
%   窗口1: 实时轨迹动画 - 显示载体运动轨迹
%   窗口2: 状态量与参数动画 (3×3布局):
%          第一行: 姿态角、速度、位置
%          第二行: 陀螺仪、加速度计、里程计比例因子
%          第三行: 陀螺零偏、加计零偏、安装角
%
% 注意事项:
%   - 轨迹图会显示从数据开始到当前时刻的完整轨迹
%   - 参数图只显示从指定开始时间之后的数据
%   - 如果start_time超出数据范围，会自动调整到有效范围内
%
% 作者: [您的姓名]
% 日期: [日期]
% 版本: 3.0

% 参数检查和默认值设置
if nargin < 3 || isempty(play_speed)
    play_speed = 50;  % 默认播放速度
end

if nargin < 4 || isempty(start_time)
    start_time = 0;   % 默认从头开始播放
end

% 参数验证
if play_speed <= 0
    warning('播放速度必须为正数，使用默认值50');
    play_speed = 50;
end

% 找到开始播放的索引
data_len = size(log.pos, 1);
time_data = data.imu.tow;

% 调整开始时间到数据范围内
if start_time < time_data(1)
    fprintf('警告：指定开始时间 %.2f s 小于数据起始时间 %.2f s，从数据起始时间开始播放\n', ...
            start_time, time_data(1));
    start_time = time_data(1);
elseif start_time > time_data(end)
    fprintf('警告：指定开始时间 %.2f s 大于数据结束时间 %.2f s，从数据结束前开始播放\n', ...
            start_time, time_data(end));
    start_time = time_data(end) - 10; % 从结束前10秒开始
    if start_time < time_data(1)
        start_time = time_data(1);
    end
end

% 找到最接近开始时间的索引
[~, start_idx] = min(abs(time_data - start_time));
actual_start_time = time_data(start_idx);

%% 创建第一个窗口：轨迹动画
fig1 = figure('Name', '实时轨迹动画', 'Position', [50, 100, 1200, 800]);
h_traj = animatedline('Color', '#D95319', 'LineWidth', 3);
hold on;

% 显示从开始到起始播放点的轨迹（灰色虚线）
if start_idx > 1
    h_traj_history = animatedline('Color', [0.7 0.7 0.7], 'LineWidth', 2, 'LineStyle', '--');
    for j = 1:start_idx
        addpoints(h_traj_history, log.pos(j, 1), log.pos(j, 2));
    end
end

h_current = plot(log.pos(start_idx,1), log.pos(start_idx,2), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black', 'LineWidth', 2);

% 添加起点、播放起始点和终点标记
plot(log.pos(1,1), log.pos(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
if start_idx > 1
    plot(log.pos(start_idx,1), log.pos(start_idx,2), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'magenta', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
end
plot(log.pos(end,1), log.pos(end,2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black', 'LineWidth', 2);

xlabel('东向 (m)', 'FontSize', 14);
ylabel('北向 (m)', 'FontSize', 14);
title('实时轨迹动画', 'FontSize', 18, 'FontWeight', 'bold');

% 根据是否有历史轨迹调整图例
if start_idx > 1
    legend({'动态轨迹', '历史轨迹', '当前位置', '数据起点', '播放起点', '数据终点'}, 'Location', 'best', 'FontSize', 12);
else
    legend({'轨迹', '当前位置', '起点', '终点'}, 'Location', 'best', 'FontSize', 12);
end
grid on;
axis equal;

% 设置坐标轴范围
pos_range_x = [min(log.pos(:,1))-50, max(log.pos(:,1))+50];
pos_range_y = [min(log.pos(:,2))-50, max(log.pos(:,2))+50];
xlim(pos_range_x);
ylim(pos_range_y);

%% 创建第二个窗口：状态量和参数 (3×3布局)
fig2 = figure('Name', '状态量与参数动画', 'Position', [1300, 100, 1600, 1000]);

% 播放参数
R2D = 180/pi;
GRAVITY = 9.8; % 重力加速度

% 显示播放信息
fprintf('=== 动态显示参数 ===\n');
fprintf('数据长度: %d 个历元\n', data_len);
fprintf('数据时间范围: %.2f - %.2f 秒\n', time_data(1), time_data(end));
fprintf('播放起始时间: %.2f 秒 (索引: %d)\n', actual_start_time, start_idx);
fprintf('播放速度: %d (每次跳过%d个点)\n', play_speed, play_speed);
fprintf('预计播放时间: %.1f 秒\n', (data_len-start_idx+1)/play_speed/10);
fprintf('==================\n');

%% 第一行：姿态、速度、位置
% 姿态角
subplot(3, 3, 1);
h_roll = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_pitch = animatedline('Color', '#D95319', 'LineWidth', 2);
h_yaw = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('角度 (deg)', 'FontSize', 11);
title('姿态角', 'FontSize', 13, 'FontWeight', 'bold');
legend({'Roll', 'Pitch', 'Yaw'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 速度
subplot(3, 3, 2);
h_vel_e = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_vel_n = animatedline('Color', '#D95319', 'LineWidth', 2);
h_vel_u = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('速度 (m/s)', 'FontSize', 11);
title('速度', 'FontSize', 13, 'FontWeight', 'bold');
legend({'East', 'North', 'Up'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 位置
subplot(3, 3, 3);
h_pos_e = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_pos_n = animatedline('Color', '#D95319', 'LineWidth', 2);
h_pos_u = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('位置 (m)', 'FontSize', 11);
title('位置', 'FontSize', 13, 'FontWeight', 'bold');
legend({'East', 'North', 'Up'}, 'Location', 'best', 'FontSize', 9);
grid on;

%% 第二行：陀螺仪、加速度计、里程计比例因子
% 陀螺仪
subplot(3, 3, 4);
h_gyro_x = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_gyro_y = animatedline('Color', '#D95319', 'LineWidth', 2);
h_gyro_z = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('角速度 (deg/s)', 'FontSize', 11);
title('陀螺仪', 'FontSize', 13, 'FontWeight', 'bold');
legend({'X轴', 'Y轴', 'Z轴'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 加速度计
subplot(3, 3, 5);
h_acc_x = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_acc_y = animatedline('Color', '#D95319', 'LineWidth', 2);
h_acc_z = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('加速度 (m/s²)', 'FontSize', 11);
title('加速度计', 'FontSize', 13, 'FontWeight', 'bold');
legend({'X轴', 'Y轴', 'Z轴'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 里程计比例因子
subplot(3, 3, 6);
h_od_scale = animatedline('Color', '#7E2F8E', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('比例因子', 'FontSize', 11);
title('里程计比例因子', 'FontSize', 13, 'FontWeight', 'bold');
grid on;

%% 第三行：陀螺零偏、加计零偏、安装角
% 陀螺零偏
subplot(3, 3, 7);
h_gyr_bias_x = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_gyr_bias_y = animatedline('Color', '#D95319', 'LineWidth', 2);
h_gyr_bias_z = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('陀螺零偏 (deg/h)', 'FontSize', 11);
title('陀螺零偏', 'FontSize', 13, 'FontWeight', 'bold');
legend({'X轴', 'Y轴', 'Z轴'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 加计零偏
subplot(3, 3, 8);
h_acc_bias_x = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_acc_bias_y = animatedline('Color', '#D95319', 'LineWidth', 2);
h_acc_bias_z = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('加计零偏 (mg)', 'FontSize', 11);
title('加计零偏', 'FontSize', 13, 'FontWeight', 'bold');
legend({'X轴', 'Y轴', 'Z轴'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 安装角
subplot(3, 3, 9);
h_install_pitch = animatedline('Color', '#0072BD', 'LineWidth', 2);
hold on;
h_install_roll = animatedline('Color', '#D95319', 'LineWidth', 2);
h_install_yaw = animatedline('Color', '#EDB120', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11);
ylabel('安装角 (deg)', 'FontSize', 11);
title('安装角', 'FontSize', 13, 'FontWeight', 'bold');
legend({'Pitch', 'Roll', 'Yaw'}, 'Location', 'best', 'FontSize', 9);
grid on;

% 创建进度条
h_wait = waitbar(0, sprintf('动画播放初始化... (播放速度: %d, 起始时间: %.2f s)', play_speed, actual_start_time), 'Name', '播放进度');

%% 动态播放
for i = start_idx:play_speed:data_len
    % 计算播放进度（基于播放部分）
    play_progress = (i - start_idx + 1) / (data_len - start_idx + 1);
    
    % 更新进度条
    if ishandle(h_wait)
        waitbar(play_progress, h_wait, sprintf('播放中... %.1f%% - 时间: %.2f s (速度: %d)', ...
                play_progress*100, data.imu.tow(i), play_speed));
    else
        break;
    end
    
    % 检查窗口是否还存在
    if ~ishandle(fig1) && ~ishandle(fig2)
        break;
    end
    
    %% 更新轨迹图（如果窗口存在）
    if ishandle(fig1)
        figure(fig1);
        addpoints(h_traj, log.pos(i, 1), log.pos(i, 2));
        set(h_current, 'XData', log.pos(i, 1), 'YData', log.pos(i, 2));
        title(sprintf('实时轨迹动画 - 时间: %.2f s (速度: %dx, 起始: %.2f s)', ...
                data.imu.tow(i), play_speed, actual_start_time), 'FontSize', 18, 'FontWeight', 'bold');
    end
    
    %% 更新状态参数图（如果窗口存在）
    if ishandle(fig2)
        figure(fig2);
        
        %% 第一行：姿态、速度、位置
        % 更新姿态角
        addpoints(h_roll, data.imu.tow(i), log.roll(i) * R2D);
        addpoints(h_pitch, data.imu.tow(i), log.pitch(i) * R2D);
        addpoints(h_yaw, data.imu.tow(i), log.yaw(i) * R2D);
        
        % 更新速度
        addpoints(h_vel_e, data.imu.tow(i), log.vel(i, 1));
        addpoints(h_vel_n, data.imu.tow(i), log.vel(i, 2));
        addpoints(h_vel_u, data.imu.tow(i), log.vel(i, 3));
        
        % 更新位置
        addpoints(h_pos_e, data.imu.tow(i), log.pos(i, 1));
        addpoints(h_pos_n, data.imu.tow(i), log.pos(i, 2));
        addpoints(h_pos_u, data.imu.tow(i), log.pos(i, 3));
        
        %% 第二行：陀螺仪、加速度计、里程计比例因子
        % 更新陀螺仪数据
        addpoints(h_gyro_x, data.imu.tow(i), data.imu.gyr(i, 1) * R2D);
        addpoints(h_gyro_y, data.imu.tow(i), data.imu.gyr(i, 2) * R2D);
        addpoints(h_gyro_z, data.imu.tow(i), data.imu.gyr(i, 3) * R2D);
        
        % 更新加速度计数据
        addpoints(h_acc_x, data.imu.tow(i), data.imu.acc(i, 1));
        addpoints(h_acc_y, data.imu.tow(i), data.imu.acc(i, 2));
        addpoints(h_acc_z, data.imu.tow(i), data.imu.acc(i, 3));
        
        % 更新里程计比例因子
        addpoints(h_od_scale, data.imu.tow(i), log.od_scale_factor(i, 1));
        
        %% 第三行：陀螺零偏、加计零偏、安装角
        % 更新陀螺零偏 (转换为deg/h)
        addpoints(h_gyr_bias_x, data.imu.tow(i), log.gyr_bias(i, 1) * R2D * 3600);
        addpoints(h_gyr_bias_y, data.imu.tow(i), log.gyr_bias(i, 2) * R2D * 3600);
        addpoints(h_gyr_bias_z, data.imu.tow(i), log.gyr_bias(i, 3) * R2D * 3600);
        
        % 更新加计零偏 (转换为mg)
        addpoints(h_acc_bias_x, data.imu.tow(i), log.acc_bias(i, 1) * 1000 / GRAVITY);
        addpoints(h_acc_bias_y, data.imu.tow(i), log.acc_bias(i, 2) * 1000 / GRAVITY);
        addpoints(h_acc_bias_z, data.imu.tow(i), log.acc_bias(i, 3) * 1000 / GRAVITY);
        
        % 更新安装角 (pitch, roll, yaw)
        addpoints(h_install_pitch, data.imu.tow(i), log.installangle(i, 1));
        addpoints(h_install_roll, data.imu.tow(i), log.installangle(i, 2));
        addpoints(h_install_yaw, data.imu.tow(i), log.installangle(i, 3));
    end
    
    % 刷新显示
    drawnow limitrate;
end

% 关闭进度条
if ishandle(h_wait)
    close(h_wait);
end

% 添加完成标题
if ishandle(fig1)
    figure(fig1);
    title('实时轨迹动画 - 播放完成', 'FontSize', 18, 'FontWeight', 'bold');
end

if ishandle(fig2)
    figure(fig2);
    sgtitle('状态量与参数动画 - 播放完成', 'FontSize', 16, 'FontWeight', 'bold');
end

fprintf('动态显示完成！播放速度: %d, 起始时间: %.2f s\n', play_speed, actual_start_time);
end
