function plot_P(time, P)
%P阵收敛曲线
    figure('name','P阵收敛曲线');
    subplot(3,2,1);
    semilogy(time, P(:, 1:3) * 180/pi, 'linewidth', 1.5); grid on;
    xlim([time(1) time(end)]);
    xlabel('时间(s)'); ylabel('平台失准角(°)'); legend('Pitch', 'Roll', 'Yaw');
    subplot(3,2,2);
    semilogy(time, P(:, 4:6), 'linewidth', 1.5); grid on;
    xlim([time(1) time(end)]);
    xlabel('时间(s)'); ylabel('速度误差(m/s)'); legend('Ve', 'Vn', 'Vu');
    subplot(3,2,4);
    semilogy(time, P(:, 7:9), 'linewidth', 1.5); grid on;
    xlim([time(1) time(end)]);
    xlabel('时间(s)'); ylabel('位置误差(m)'); legend('Lat', 'Lon', 'Alt');
    subplot(3,2,3);
    semilogy(time, P(:, 10:12) * 3600 * 180/pi, 'linewidth', 1.5); grid on;
    xlim([time(1) time(end)]);
    xlabel('时间(s)'); ylabel('陀螺零偏(°/h)'); legend('X', 'Y', 'Z');
    subplot(3,2,5);
    semilogy(time, P(:, 13:15) / 9.8 * 1000, 'linewidth', 1.5); grid on;
    xlim([time(1) time(end)]);
    xlabel('时间(s)'); ylabel('加速度计零偏(mg)'); legend('X', 'Y', 'Z');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end

