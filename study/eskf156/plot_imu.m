function plot_imu(gyro, acc, dt)
%IMU原始数据曲线
    data_length = length(gyro);
    dt = dt/60;

    figure('Name','IMU原始数据曲线');

    subplot(2,1,1);
    plot((1:data_length)*dt, gyro(:,1)); hold on; grid on;
    plot((1:data_length)*dt, gyro(:,2));
    plot((1:data_length)*dt, gyro(:,3));
    xlim([1 data_length*dt]);
    legend('X','Y','Z', 'Orientation','horizontal');
    xlabel('时间(分钟)');
    ylabel('角速度(°/s)');

    subplot(2,1,2);
    acc = acc/9.8;
    plot((1:data_length)*dt, acc(:,1)); hold on; grid on;
    plot((1:data_length)*dt, acc(:,2));
    plot((1:data_length)*dt, acc(:,3));
    xlim([1 data_length*dt]);
    legend('X','Y','Z', 'Orientation','horizontal');
    xlabel('时间(分钟)');
    ylabel('加速度(g)');

    sgtitle('IMU原始数据曲线');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end

