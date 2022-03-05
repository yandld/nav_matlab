function plot_enu_cmp(enu, enu_ref)
    figure('name', '二维轨迹对比');
    plot(enu(:,1)/1e3, enu(:,2)/1e3); hold on;
    plot(enu_ref(:,1)/1e3, enu_ref(:,2)/1e3); hold on;
    plot(enu_ref(:,1)/1e3, enu_ref(:,2)/1e3, '.');
    axis equal; grid on;
    legend('KF', 'GNSS');
    xlabel('East(km)');
    ylabel('North(km)');
    title('二维轨迹对比');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
end

