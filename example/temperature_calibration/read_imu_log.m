function [temperature, accel, gyro, gyro_z_50dps] = read_imu_log(filename)
    % Read IMU calibration log file and return data arrays
    
    if nargin < 1
        filename = 'test.log';
    end
    
    % Initialize data arrays
    temperature = [];
    accel_data = [];
    gyro_data = [];
    gyro_z_50dps_data = [];
    
    % Read file
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    % Parse each line
    while ~feof(fid)
        line = fgetl(fid);
        if contains(line, 'BIN_DATA:')
            % Fixed regex pattern - escape backslashes properly
            data = regexp(line, '[-+]?\d+\.?\d*', 'match');
            
            % Debug: show what was extracted
            if isempty(data)
                fprintf('No match found in line: %s\n', line);
            else
                fprintf('Found %d numbers\n', length(data));
            end
            
            if length(data) >= 10
                temperature(end+1) = str2double(data{2});
                accel_data(end+1,:) = [str2double(data{5}), str2double(data{6}), str2double(data{7})];
                gyro_data(end+1,:) = [str2double(data{8}), str2double(data{9}), str2double(data{10})];
                
                if length(data) >= 11
                    gyro_z_50dps_data(end+1) = str2double(data{11});
                else
                    gyro_z_50dps_data(end+1) = NaN;
                end
            end
        end
    end
    fclose(fid);
    
    % Return as column vector and matrices
    temperature = temperature';
    accel = accel_data;
    gyro = gyro_data;
    gyro_z_50dps = gyro_z_50dps_data';
    
    fprintf('Read %d data points\n', length(temperature));
end
