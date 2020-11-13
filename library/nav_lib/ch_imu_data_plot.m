%% Example:
% subplot：是否开启subplot
function ch_imu_data_plot(varargin)
%%  plot imu data
i = 1;
param= inputParser;
param.addOptional('time', []);
param.addOptional('pos_fusion', []);
param.addOptional('pos_gnss', []);
param.addOptional('acc', []);
param.addOptional('gyr', []);
param.addOptional('mag', []);
param.addOptional('eul', []);
param.addOptional('gb', []); % 加速度零偏
param.addOptional('wb', []); % 陀螺零偏
param.addOptional('phi', []); %失准角
param.addOptional('P_phi', []); %失准角方差
param.addOptional('P_wb', []); %陀螺方差
param.addOptional('P_pos', []); %位置方差
param.addOptional('subplot', []);


%然后将输入的参数进行处理，如果有不同于默认值的那就覆盖掉
param.parse(varargin{:});
r = param.Results;

if(r.time == 0 )
    error('no time data');
end

figure('Name', 'Sensor Data');

if(~isempty(r.pos_fusion))
    subplot(2,1,1);
    
    h=zeros(1,3);
    plot(r.pos_gnss(:,2), r.pos_gnss(:,1),'b-');
    hold on;
    h(1) = plot(r.pos_gnss(:,2), r.pos_gnss(:,1),'b.');
    h(2) = plot(r.pos_fusion(:,2), r.pos_fusion(:,1), 'r-');
    h(3) = plot(r.pos_fusion(1,1), r.pos_fusion(1,2),'ks');
    legend(h,'GNSS position estimate','GNSS aided INS trajectory','Start point')
    axis equal
    hold off;
    xlabel('X(m)'); ylabel('Y(m)');  title('Trajectory');

    subplot(2,1,2);
    h=zeros(1,2);
    hold on;
    h(1) = plot(1:length(r.pos_gnss), -r.pos_gnss(:,3),'b.');
    h(2) = plot(r.time, -r.pos_fusion(:,3),'r');
    legend(h, 'GNSS estimate','GNSS aided INS estimate')
    title('Height versus time');  xlabel('Time [s]');  ylabel('Height [m]');
    hold off;
    i = i+1;
end

if(~isempty(r.gyr))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.gyr, {'X', 'Y', 'Z'}, 'Time (s)', 'Angular rate (dps(deg /s))', 'Gyroscope');
    i = i+1;
end

if(~isempty(r.acc))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.acc, {'X', 'Y', 'Z'}, 'Time (s)', 'Acceleration (g)', 'Accelerometer');
    i = i+1;
end

if(~isempty(r.mag))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.mag, {'X', 'Y', 'Z'}, 'Time (s)', 'Flux (G)', 'Magnetometer');
    i = i+1;
end

if(~isempty(r.eul))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.eul, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', 'Eular Angle');
    i = i+1;
end

if(~isempty(r.wb))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.wb, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', '陀螺零偏');
    i = i+1;
end

if(~isempty(r.gb))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.gb, {'X', 'Y', 'Z'}, 'Time (s)', 'm/s^(2)', '加速度零偏');
    i = i+1;
end

if(~isempty(r.phi))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.phi, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', '失准角');
    i = i+1;
end

if(~isempty(r.P_phi))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.P_phi, {'X', 'Y', 'Z'}, 'Time (s)', '-', 'Phi Var(失准角方差)');
    i = i+1;
end


if(~isempty(r.P_wb))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.P_wb, {'X', 'Y', 'Z'}, 'Time (s)', '-', '陀螺零偏方差');
    i = i+1;
end


if(~isempty(r.P_pos))
    if(r.subplot == 1)
        subplot(2,2,i);
    else
        if i ~= 1; figure; end
    end
    interial_display(r.time,  r.P_pos, {'X', 'Y', 'Z'}, 'Time (s)', '-', '位置方差');
    i = i+1;
end

%    linkaxes(axis, 'x');

end


function interial_display(time, data, legendstr, xlabelstr, ylabelstr, titlestr)
hold on;
plot(time, data(:,1), 'r');
plot(time, data(:,2), 'g');
plot(time, data(:,3), 'b');
legend(legendstr);
xlabel(xlabelstr);
ylabel(ylabelstr);
title(titlestr);
hold off;
end
