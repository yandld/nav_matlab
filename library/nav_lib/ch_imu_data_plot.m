function ch_imu_data_plot(varargin)
%%  plot imu data

	param= inputParser; 
	param.addOptional('time', []);
    param.addOptional('acc', []);
    param.addOptional('gyr', []);
    param.addOptional('mag', []);
    param.addOptional('eul', []);
    
    %然后将输入的参数进行处理，如果有不同于默认值的那就覆盖掉
    param.parse(varargin{:});
    r = param.Results;


    if(r.time == 0 )
           error('no time data');
    end

    figure('Name', 'Sensor Data');
    
    if(~isempty(r.gyr))
         axis(1) = subplot(2,2,1);
         hold on;
        plot(r.time, r.gyr(:,1), 'r');
        plot(r.time, r.gyr(:,2), 'g');
        plot(r.time, r.gyr(:,3), 'b');
        legend('X', 'Y', 'Z');
        xlabel('Time (s)');
        ylabel('Angular rate (deg/s)');
        title('Gyroscope');
        hold off;
    end

    if(~isempty(r.acc))
        axis(2) = subplot(2,2,2);
        hold on;
        plot(r.time, r.acc(:,1), 'r');
        plot(r.time, r.acc(:,2), 'g');
        plot(r.time, r.acc(:,3), 'b');
        legend('X', 'Y', 'Z');
        xlabel('Time (s)');
        ylabel('Acceleration (g)');
        title('Accelerometer');
        hold off;
    end
    
    if(~isempty(r.mag))
        axis(3) = subplot(2,2,3);
        hold on;
        plot(r.time, r.mag(:,1), 'r');
        plot(r.time, r.mag(:,2), 'g');
        plot(r.time, r.mag(:,3), 'b');
        legend('X', 'Y', 'Z');
        xlabel('Time (s)');
        ylabel('Flux (G)');
        title('Magnetometer');
        hold off;
	end

    if(~isempty(r.eul))
        axis(4) = subplot(2,2,4);
        hold on;
        plot(r.time, r.eul(:,1), 'r');
        plot(r.time,  r.eul(:,2), 'g');
        plot(r.time,  r.eul(:,3), 'b');
        legend('Y', 'P', 'R');
        xlabel('Time (s)');
        ylabel('Angle(°)');
        title('Eular Angle');
     hold off;
    end


    linkaxes(axis, 'x');

end
        