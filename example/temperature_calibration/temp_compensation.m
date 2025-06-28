function comp_params = temp_compensation(temp, accel, gyro)
    % IMU temperature drift coefficient calculation
    % Direct fitting: sensor_output = bias + a1*T + a2*T^2
    
    % Design matrix with constant term
    A = [ones(length(temp), 1), temp(:), temp(:).^2];
    
    % Initialize compensation parameters
    comp_params.accel = zeros(3, 3);  % [bias, linear, quadratic]
    comp_params.gyro = zeros(3, 3);
    
    % Fit coefficients for each axis
    for axis = 1:3
        comp_params.accel(axis, :) = A \ accel(:, axis);
        comp_params.gyro(axis, :) = A \ gyro(:, axis);
    end
    
    % Display compensation parameters with XYZ labels
    fprintf('=== Temperature Compensation Coefficients ===\n');
    fprintf('\nAccelerometer: bias + a1*T + a2*T²\n');
    
    axis_labels = {'X', 'Y', 'Z'};
    for axis = 1:3
        fprintf('  Axis %s: %.6f + %.6f*T + %.8f*T²\n', ...
                axis_labels{axis}, ...
                comp_params.accel(axis, 1), ...  % bias
                comp_params.accel(axis, 2), ...  % linear
                comp_params.accel(axis, 3));     % quadratic
    end
    
    fprintf('\nGyroscope: bias + a1*T + a2*T²\n');
    for axis = 1:3
        fprintf('  Axis %s: %.6f + %.6f*T + %.8f*T²\n', ...
                axis_labels{axis}, ...
                comp_params.gyro(axis, 1), ...   % bias
                comp_params.gyro(axis, 2), ...   % linear
                comp_params.gyro(axis, 3));      % quadratic
    end
    
    % Extract coefficients for C code format
    fprintf('\n=== C Code Format ===\n');
    fprintf('// Accelerometer coefficients\n');
    fprintf('acc_bias: [%.6f, %.6f, %.6f]\n', comp_params.accel(:, 1)');
    fprintf('acc_tc1:  [%.6f, %.6f, %.6f]\n', comp_params.accel(:, 2)');
    fprintf('acc_tc2:  [%.8f, %.8f, %.8f]\n', comp_params.accel(:, 3)');
    
    fprintf('// Gyroscope coefficients\n');
    fprintf('gyr_bias: [%.6f, %.6f, %.6f]\n', comp_params.gyro(:, 1)');
    fprintf('gyr_tc1:  [%.6f, %.6f, %.6f]\n', comp_params.gyro(:, 2)');
    fprintf('gyr_tc2:  [%.8f, %.8f, %.8f]\n', comp_params.gyro(:, 3)');
end
