% subplot：是否开启subplot
function ch_plot_imu(varargin)
%%  plot imu data
i = 1;
param= inputParser;
param.addOptional('time', []);
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
param.addOptional('title', []);
param.addOptional('legend', []);


%然后将输入的参数进行处理，如果有不同于默认值的那就覆盖掉
param.parse(varargin{:});
r = param.Results;

if(r.time == 0 )
    error('no time data');
end
i = 1;

figure;

if(~isempty(r.gyr))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.gyr, {'X', 'Y', 'Z'}, 'Time (s)', 'Angular rate (dps(deg /s))', 'Gyroscope');
end

if(~isempty(r.acc))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.acc, {'X', 'Y', 'Z'}, 'Time (s)', 'Acceleration (g)', 'Accelerometer');
end

if(~isempty(r.mag))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.mag, {'X', 'Y', 'Z'}, 'Time (s)', 'Flux (G)', 'Magnetometer');
end

if(~isempty(r.eul))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.eul, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', 'Eular Angle');
end

if(~isempty(r.wb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.wb, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', '陀螺零偏');
end

if(~isempty(r.gb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.gb, {'X', 'Y', 'Z'}, 'Time (s)', 'm/s^(2)', '加速度零偏');
end

if(~isempty(r.phi))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.phi, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', '失准角');
end

if(~isempty(r.P_phi))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.P_phi, {'X', 'Y', 'Z'}, 'Time (s)', '-', 'Phi Var(失准角方差)');
end


if(~isempty(r.P_wb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.P_wb, {'X', 'Y', 'Z'}, 'Time (s)', '-', '陀螺零偏方差');
end


if(~isempty(r.P_pos))
    subplot(2,2,i);
    interial_display(r.time,  r.P_pos, {'X', 'Y', 'Z'}, 'Time (s)', '-', '位置方差');
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
