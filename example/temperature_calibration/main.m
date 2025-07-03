clear;
clc;
close all;

temp_scale = 100;

% Read IMU data (now includes Z-axis 50dps data)
[temp, accel, gyro, gyro_z_50dps] = read_imu_log('test.log');

% Remove 1G bias from Z-axis accelerometer
accel(:,3) = accel(:,3) - 9.8;

% Scale temperature
temp = temp / temp_scale;

% Calculate pure temperature drift coefficients
comp_params = temp_compensation(temp, accel, gyro);

% Add Z-axis scale factor compensation
z_scale_params = calculate_z_scale_compensation(temp*temp_scale, gyro_z_50dps);

% Apply both compensations
[accel_comp, gyro_comp] = apply_temp_compensation(temp, accel, gyro, comp_params, z_scale_params);

% Create comparison plots
figure('Position', [100, 100, 1200, 800]);

% Plot comparisons with unified scales
plot_sensor_comparison(1, temp*temp_scale, accel, accel_comp, 'Original Accelerometer', 'Compensated Accelerometer', 'Acceleration (g)');
plot_sensor_comparison(3, temp*temp_scale, gyro*57.3, gyro_comp*57.3, 'Original Gyroscope', 'Compensated Gyroscope', 'Angular Velocity (deg/s)');

% Plot Z-axis scale factor compensation
plot_z_scale_factor_comparison(temp*temp_scale, gyro_z_50dps, z_scale_params);

% Validation results
validate_sensor(accel, accel_comp, 'Accelerometer');
validate_sensor(gyro*57.3, gyro_comp*57.3, 'Gyroscope');




function z_params = calculate_z_scale_compensation(temp, gyro_z_50dps)
    % Calculate Z-axis scale factor temperature compensation
    % Based on 50 dps reference rotation
    
    expected_rate = 50;  % 50 dps in rad/s
    ref_temp = 0.25;              % Reference temperature (25°C scaled)
    
    % Remove zero entries (when no rotation applied)
    valid_idx = gyro_z_50dps ~= 0;
    temp_valid = temp(valid_idx);
    gyro_valid = gyro_z_50dps(valid_idx);
    
    if length(temp_valid) < 3
        warning('Insufficient Z-axis calibration data');
        z_params.tc1 = 0;
        z_params.tc2 = 0;
        return;
    end
    
    % Calculate scale factor error
    scale_error = abs(gyro_valid) / expected_rate;
    
    % Design matrix: [1, T, T²]
    A = [ones(length(temp_valid), 1), temp_valid, temp_valid.^2];
    
    % Solve least squares
    coeffs = A \ scale_error;
    
    z_params.bias = coeffs(1);
    z_params.tc1 = coeffs(2);
    z_params.tc2 = coeffs(3);
    z_params.ref_temp = ref_temp;
    
    fprintf('\n=== Z-axis Scale Factor Compensation ===\n');
    fprintf('Scale bias: %.6f\n', z_params.bias);
    fprintf('Linear TC1: %.6f (1/°C)\n', z_params.tc1);
    fprintf('Quadratic TC2: %.6f (1/°C²)\n', z_params.tc2);
end

function [accel_comp, gyro_comp] = apply_temp_compensation(temp, accel, gyro, comp_params, z_scale_params)
    % Apply temperature compensation - remove fitted temperature response
    
    % Original drift compensation
    A = [ones(length(temp), 1), temp(:), temp(:).^2];
    
    accel_predicted = A * comp_params.accel';
    gyro_predicted = A * comp_params.gyro';
    
    % Remove temperature drift
    accel_comp = accel - accel_predicted;
    gyro_comp = gyro - gyro_predicted;
    
    % Apply Z-axis scale factor compensation
    if nargin > 4 && ~isempty(z_scale_params)
        ref_temp = z_scale_params.ref_temp;
        
        for i = 1:length(temp)
            temp_diff = temp(i) - ref_temp;
            
            % Scale factor compensation: rate / (1 + tc1*dT + tc2*dT²)
            compensation_factor = 1 + z_scale_params.tc1 * temp_diff + ...
                                 z_scale_params.tc2 * (temp(i)^2 - ref_temp^2);
            
            gyro_comp(i, 3) = gyro_comp(i, 3) / compensation_factor;
        end
        
        fprintf('Applied Z-axis scale factor compensation\n');
    end
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


function plot_z_scale_factor_comparison(temp_celsius, gyro_z_50dps, z_scale_params)
    % Plot Z-axis scale factor compensation comparison
    
    expected_rate = 50 * pi/180;  % 50 dps in rad/s
    
    % Remove zero entries (when no rotation applied)
    valid_idx = gyro_z_50dps ~= 0;

    temp_valid = temp_celsius(valid_idx);
    gyro_z_valid = gyro_z_50dps(valid_idx);
    
    % Calculate original scale factor error (percentage)
    scale_error_orig = (abs(gyro_z_valid) / expected_rate - 1) * 100;
    
    % Apply scale factor compensation
    ref_temp = z_scale_params.ref_temp;
    
    compensation_factor = 1 + z_scale_params.tc1 * (temp_valid - ref_temp) + ...
                         z_scale_params.tc2 * (temp_valid.^2 - ref_temp^2);
    gyro_z_comp = gyro_z_valid ./ compensation_factor;
    scale_error_comp = (abs(gyro_z_comp) / expected_rate - 1) * 100;
    
    % Calculate unified Y-axis limits
    all_errors = [scale_error_orig; scale_error_comp];
    y_min = min(all_errors);
    y_max = max(all_errors);
    y_margin = (y_max - y_min) * 0.1;  % 10% margin
    y_limits = [y_min - y_margin, y_max + y_margin];
    
    % Create figure
    figure('Position', [200, 200, 1200, 400]);
    
    % Original data
    subplot(1,2,1);
    plot(temp_valid, scale_error_orig, 'b.-', 'LineWidth', 1.0, 'MarkerSize', 6);
    xlabel('Temperature (°C)'); 
    ylabel('Scale Factor Error (%)'); 
    title('Original Z-axis Scale Factor Error');
    grid on;
    ylim(y_limits);
    
    % Compensated data
    subplot(1,2,2);
    plot(temp_valid, scale_error_comp, 'b.-', 'LineWidth', 1.0, 'MarkerSize', 6);
    xlabel('Temperature (°C)'); 
    ylabel('Scale Factor Error (%)'); 
    title('Compensated Z-axis Scale Factor Error');
    grid on;
    ylim(y_limits);
    
    sgtitle('Z-axis Gyroscope Scale Factor Temperature Compensation', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Print validation results
    orig_std = std(scale_error_orig);
    comp_std = std(scale_error_comp);
    improvement = (orig_std - comp_std) / orig_std * 100;
    
    fprintf('\n=== Z-axis Scale Factor Validation ===\n');
    fprintf('Original Error Std: %.4f%% \n', orig_std);
    fprintf('Compensated Error Std: %.4f%% \n', comp_std);
    fprintf('Scale Factor Improvement: %.1f%%\n', improvement);
end

