clear;
clc;
close all;

% Read IMU data
[temp, accel, gyro] = read_imu_log('test.log');

% Remove 1G bias from Z-axis accelerometer
accel(:,3) = accel(:,3) - 9.8;

% gyro = gyro *57.3;
temp = temp / 100;

% Calculate pure temperature drift coefficients
comp_params = temp_compensation(temp, accel, gyro);


[accel_comp, gyro_comp] = apply_temp_compensation(temp, accel, gyro, comp_params);
% Create comparison plots with unified Y-axis
figure('Position', [100, 100, 1200, 800]);


function [accel_comp, gyro_comp] = apply_temp_compensation(temp, accel, gyro, comp_params)
    % Apply temperature compensation - remove fitted temperature response
    
    % Calculate predicted temperature-dependent response
    A = [ones(length(temp), 1), temp(:), temp(:).^2];
    
    accel_predicted = A * comp_params.accel';
    gyro_predicted = A * comp_params.gyro';
    
    % Remove temperature effect (keep only residuals)
    accel_comp = accel - accel_predicted;
    gyro_comp = gyro - gyro_predicted;
end



% Modified plot function with unified Y-axis
function plot_sensor_comparison(subplot_start, temp, orig_data, comp_data, title_orig, title_comp, ylabel_str)
    % Calculate unified Y-axis limits
    all_data = [orig_data(:); comp_data(:)];
    y_min = min(all_data);
    y_max = max(all_data);
    y_margin = (y_max - y_min) * 0.1;  % 10% margin
    y_limits = [y_min - y_margin, y_max + y_margin];
    
    % Original data
    subplot(2,2,subplot_start);
    plot(temp, orig_data(:,1), 'r.-', temp, orig_data(:,2), 'g.-', temp, orig_data(:,3), 'b.-', ...
         'LineWidth', 1.0, 'MarkerSize', 6);
    xlabel('Temperature (°C)'); ylabel(ylabel_str); title(title_orig);
    legend('X', 'Y', 'Z', 'Location', 'best'); grid on;
    ylim(y_limits);  % Set unified Y-axis limits
    
    % Compensated data
    subplot(2,2,subplot_start+1);
    plot(temp, comp_data(:,1), 'r.-', temp, comp_data(:,2), 'g.-', temp, comp_data(:,3), 'b.-', ...
         'LineWidth', 1.0, 'MarkerSize', 6);
    xlabel('Temperature (°C)'); ylabel(ylabel_str); title(title_comp);
    legend('X', 'Y', 'Z', 'Location', 'best'); grid on;
    ylim(y_limits);  % Set unified Y-axis limits
end

% Validation function with XYZ labels
function validate_sensor(orig_data, comp_data, sensor_name)
    fprintf('\n=== %s Validation ===\n', sensor_name);
    axis_labels = {'X', 'Y', 'Z'};
    
    for axis = 1:3
        orig_std = std(orig_data(:,axis));
        comp_std = std(comp_data(:,axis));
        improvement = (orig_std-comp_std)/orig_std*100;
        
        fprintf('Axis %s: Std %.6f->%.6f (%.1f%% improvement)\n', ...
                axis_labels{axis}, orig_std, comp_std, improvement);
    end
end


% Plot comparisons with unified scales
plot_sensor_comparison(1, temp, accel, accel_comp, 'Original Accelerometer', 'Compensated Accelerometer', 'Acceleration (g)');
plot_sensor_comparison(3, temp, gyro, gyro_comp, 'Original Gyroscope', 'Compensated Gyroscope', 'Angular Velocity (deg/s)');

