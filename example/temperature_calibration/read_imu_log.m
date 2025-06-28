function [temperature, accel, gyro] = read_imu_log(filename)
    % Read IMU calibration log file and return data arrays
    
    if nargin < 1
        filename = 'test.log';
    end
    
    % Initialize data arrays
    temperature = [];
    accel_data = [];
    gyro_data = [];
    
    % Read file
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    % Parse each line
    while ~feof(fid)
        line = fgetl(fid);
        if contains(line, 'BIN_DATA:')
            % Extract numeric data
            data = regexp(line, '[-+]?\d+\.?\d*', 'match');
            if length(data) >= 9
                temperature(end+1) = str2double(data{2});
                accel_data(end+1,:) = [str2double(data{4}), str2double(data{5}), str2double(data{6})];
                gyro_data(end+1,:) = [str2double(data{7}), str2double(data{8}), str2double(data{9})];
            end
        end
    end
    fclose(fid);
    
    % Return as column vector and matrices
    temperature = temperature';
    accel = accel_data;
    gyro = gyro_data;
    
    fprintf('Read %d data points\n', length(temperature));
end
