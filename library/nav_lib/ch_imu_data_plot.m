    %% Example: 
     % subplot：是否开启subplot
    function ch_imu_data_plot(varargin)
    %%  plot imu data
    i = 1;
    param= inputParser; 
    param.addOptional('time', []);
    param.addOptional('acc', []);
    param.addOptional('gyr', []);
    param.addOptional('mag', []);
    param.addOptional('eul', []);
    param.addOptional('wb', []); % 陀螺零偏
    param.addOptional('phi', []); %失准角
    param.addOptional('subplot', []);

    %然后将输入的参数进行处理，如果有不同于默认值的那就覆盖掉
    param.parse(varargin{:});
    r = param.Results;

    if(r.time == 0 )
    error('no time data');
    end

    figure('Name', 'Sensor Data');

    if(~isempty(r.gyr))
    if(r.subplot == 1)
    subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.gyr(:,1), 'r');
    plot(r.time, r.gyr(:,2), 'g');
    plot(r.time, r.gyr(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');
    ylabel('Angular rate (deg/s)');
    title('Gyroscope');
    hold off;
    i = i+1;
    end

    if(~isempty(r.acc))
    if(r.subplot == 1)
     subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.acc(:,1), 'r');
    plot(r.time, r.acc(:,2), 'g');
    plot(r.time, r.acc(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)');ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
    i = i+1;
    end

    if(~isempty(r.mag))
    if(r.subplot == 1)
    subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.mag(:,1), 'r'); plot(r.time, r.mag(:,2), 'g'); plot(r.time, r.mag(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)'); ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    i = i+1;
    end

    if(~isempty(r.eul))
    if(r.subplot == 1)
    subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.eul(:,1), 'r'); plot(r.time,  r.eul(:,2), 'g'); plot(r.time,  r.eul(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)'); ylabel('Angle(deg)');
    title('Eular Angle');
    hold off;
    i = i+1;
    end

    if(~isempty(r.wb))
    if(r.subplot == 1)
    subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.wb(:,1), 'r'); plot(r.time,  r.wb(:,2), 'g'); plot(r.time,  r.wb(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)'); ylabel('Angle(deg)');
    title('Gyr Bias');
    hold off;
    i = i+1;
    end

    if(~isempty(r.phi))
    if(r.subplot == 1)
    subplot(2,2,i);
    else
    if i ~= 1; figure; end
    end
    hold on;
    plot(r.time, r.phi(:,1), 'r'); plot(r.time,  r.phi(:,2), 'g'); plot(r.time,  r.phi(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Time (s)'); ylabel('Angle(deg)');
    title('失准角');
    hold off;
    i = i+1;
    end
    
    %    linkaxes(axis, 'x');

    end
