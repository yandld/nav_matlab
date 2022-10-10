function plot_vel(gnss_time,gnss_vel, matlab_time,matlab_vel, mcu_time,mcu_vel)
%速度估计曲线
    for i=2:2:nargin
        if i==2
%             gnss_time = gnss_time/60;
            figure('name', '速度估计曲线');
            subplot(3,1,1);
            plot(gnss_time, gnss_vel(:,1), 'r', 'Marker','.'); hold on; grid on;
            xlim([gnss_time(1) gnss_time(end)]);
            ylabel('东向速度(m/s)'); legend('GNSS');
            subplot(3,1,2);
            plot(gnss_time, gnss_vel(:,2), 'r', 'Marker','.'); hold on; grid on;
            xlim([gnss_time(1) gnss_time(end)]);
            ylabel('北向速度(m/s)'); legend('GNSS');
            subplot(3,1,3);
            plot(gnss_time, gnss_vel(:,3), 'r', 'Marker','.'); hold on; grid on;
            xlim([gnss_time(1) gnss_time(end)]);
            xlabel('时间(s)'); ylabel('天向速度(m/s)'); legend('GNSS');
        elseif i==4
%             matlab_time = matlab_time/60;
            subplot(3,1,1);
            plot(matlab_time, matlab_vel(:,1), 'b', 'Marker','.');
            legend('GNSS', 'MATLAB');
            subplot(3,1,2);
            plot(matlab_time, matlab_vel(:,2), 'b', 'Marker','.');
            legend('GNSS', 'MATLAB');
            subplot(3,1,3);
            plot(matlab_time, matlab_vel(:,3), 'b', 'Marker','.');
            legend('GNSS', 'MATLAB');
        elseif i==6
%             mcu_time = mcu_time/60;
            subplot(3,1,1);
            plot(mcu_time, mcu_vel(:,1), 'm', 'Marker','.');
            legend('GNSS', 'MATLAB', 'MCU');
            subplot(3,1,2);
            plot(mcu_time, mcu_vel(:,2), 'm', 'Marker','.');
            legend('GNSS', 'MATLAB', 'MCU');
            subplot(3,1,3);
            plot(mcu_time, mcu_vel(:,3), 'm', 'Marker','.');
            legend('GNSS', 'MATLAB', 'MCU');
        end
    end

    sgtitle('速度估计曲线');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end

