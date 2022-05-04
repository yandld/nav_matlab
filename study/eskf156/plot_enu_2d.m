function plot_enu_2d(gnss_enu, matlab_enu, mcu_enu)
%二维轨迹曲线
%   此处提供详细说明
    for i=1:nargin
        if i==1
            figure('name', '二维轨迹曲线');
            plot(gnss_enu(:,1), gnss_enu(:,2), 'r', 'Marker','.'); hold on; axis equal; grid on;
            legend('GNSS');
            xlabel('East(m)');
            ylabel('North(m)');
        elseif i==2
            plot(matlab_enu(:,1), matlab_enu(:,2), 'b', 'Marker','.');
            legend('GNSS', 'MATLAB');
        elseif i==3
            plot(mcu_enu(:,1), mcu_enu(:,2), 'm', 'Marker','.');
            legend('GNSS', 'MATLAB', 'MCU');
        end
    end

    title('二维轨迹曲线');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end