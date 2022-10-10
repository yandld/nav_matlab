function plot_att(matlab_time,matlab_att, mcu_time,mcu_att, sins_time,sins_att, bl_time,bl_att)
%姿态与航向估计曲线
    for i=2:2:nargin
        if i==2
%             matlab_time = matlab_time/60;
            figure('name', '姿态与航向估计曲线');
            subplot(3,1,1);
            plot(matlab_time, matlab_att(:,1), 'b', 'linewidth', 1.5); hold on; grid on;
            xlim([matlab_time(1) matlab_time(end)]);
            ylim([-20 20]);
            ylabel('Pitch(°)'); legend('MATLAB', 'Orientation','horizontal');
            subplot(3,1,2);
            plot(matlab_time, matlab_att(:,2), 'b', 'linewidth', 1.5); hold on; grid on;
            xlim([matlab_time(1) matlab_time(end)]);
            ylim([-20 20]);
            ylabel('Roll(°)'); legend('MATLAB', 'Orientation','horizontal');
            subplot(3,1,3);
            plot(matlab_time, matlab_att(:,3), 'b', 'linewidth', 1.5); hold on; grid on;
            legend('MATLAB', 'Orientation','horizontal');
            xlim([matlab_time(1) matlab_time(end)]);
            ylim([-30 420]);
            xlabel('时间(s)'); ylabel('Yaw(°)');
        elseif i==4
%             mcu_time = mcu_time/60;
            subplot(3,1,1);
            plot(mcu_time, mcu_att(:,1), 'm', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', 'Orientation','horizontal');
            subplot(3,1,2);
            plot(mcu_time, mcu_att(:,2), 'm', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', 'Orientation','horizontal');
            subplot(3,1,3);
            plot(mcu_time, mcu_att(:,3), 'm', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', 'Orientation','horizontal');
        elseif i==6
%             sins_time = sins_time/60;
            subplot(3,1,1);
            plot(sins_time, sins_att(:,1), 'r', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', '纯惯性', 'Orientation','horizontal');
            subplot(3,1,2);
            plot(sins_time, sins_att(:,2), 'r', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', '纯惯性', 'Orientation','horizontal');
            subplot(3,1,3);
            plot(sins_time, sins_att(:,3), 'r', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', '纯惯性', 'Orientation','horizontal');
        elseif i==8
%             bl_time = bl_time/60;
            subplot(3,1,1);
            plot(bl_time, bl_att(:,1), 'g', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', '纯惯性', '双天线','Orientation','horizontal');
            subplot(3,1,3);
            plot(bl_time, bl_att(:,2), 'g', 'linewidth', 1.5);
            legend('MATLAB', 'MCU', '纯惯性', '双天线','Orientation','horizontal');
        end
    end

    sgtitle('姿态与航向估计曲线');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end

