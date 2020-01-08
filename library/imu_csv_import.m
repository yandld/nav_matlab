%% 用于读取CSV的IMU数据

% EularAngle is ZYX/YPR
function [Accelerometer, Gyroscope, Magnetometer, EularAngle, time] = imu_csv_import(path,  varargin)

    t = readtable(path);
    
    % set init value 
    len = size(t,1);
    Accelerometer = zeros(len, 3);
    Gyroscope = zeros(len, 3);
    Magnetometer = zeros(len, 3);
    EularAngle = zeros(len, 3);
    time = 1:len;
    
    % acc
    index = find(contains(t.Properties.VariableNames,'AccRaw'), 1);
    if index ~= 0
    Accelerometer(:,1) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'AccRawX'), 1);
    if index ~= 0
    Accelerometer(:,1) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'AccRawY'), 1);
    if index ~= 0
    Accelerometer(:,2) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'AccRawZ'), 1);
    if index ~= 0
    Accelerometer(:,3) = table2array(t(:,index));
    end
    
    % gyo calibrated
    index = find(contains(t.Properties.VariableNames,'GyoCalibratedX'), 1);
    if index ~= 0
    Gyroscope(:,1) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'GyoCalibratedY'), 1);
    if index ~= 0
    Gyroscope(:,2) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'GyoCalibratedZ'), 1);
    if index ~= 0
    Gyroscope(:,3) = table2array(t(:,index));
    end
    
    % gyr
    index = find(contains(t.Properties.VariableNames,'GyoRawX'), 1);
    if index ~= 0
    Gyroscope(:,1) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'GyoRawY'), 1);
    if index ~= 0
    Gyroscope(:,2) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'GyoRawZ'), 1);
    if index ~= 0
    Gyroscope(:,3) = table2array(t(:,index));
    end

    % mag
    index = find(contains(t.Properties.VariableNames,'MagRawX'), 1);
    if index ~= 0
    Magnetometer(:,1) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'MagRawY'), 1);
    if index ~= 0
    Magnetometer(:,2) = table2array(t(:,index));
    end
    
    index = find(contains(t.Properties.VariableNames,'MagRawZ'), 1);
    if index ~= 0
    Magnetometer(:,3) = table2array(t(:,index));
    end
    
    % pitch
    index = find(contains(t.Properties.VariableNames,'Pitch'), 1);
    if index ~= 0
    EularAngle(:,2) = table2array(t(:,index));
    end
    
    % roll
    index = find(contains(t.Properties.VariableNames,'Roll'), 1);
    if index ~= 0
    EularAngle(:,3) = table2array(t(:,index));
    end
    % yaw
    index = find(contains(t.Properties.VariableNames,'Yaw'), 1);
    if index ~= 0
    EularAngle(:,1) = table2array(t(:,index));
    end
    
end
