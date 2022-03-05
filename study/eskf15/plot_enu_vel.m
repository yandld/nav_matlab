function plot_enu_vel(enu, vel)
    figure('name', '二维轨迹与速度曲线');
    scatter(enu(:,1)/1e3, enu(:,2)/1e3, 5, vel*3.6, 'filled');
    ct = colorbar('southoutside');
    ct.Label.String = 'Velocity(km/h)';
    caxis([0, 120]); % 速度colorbar范围(0~120km/h)
    axis equal; grid on;
    xlabel('East(km)');
    ylabel('North(km)');
    title('二维轨迹与速度');
    set(gcf, 'Units', 'normalized', 'Position', [0.025, 0.05, 0.95, 0.85]);
    
    % hold on;
    % plot(gps_enu(:,1)/1e3, gps_enu(:,2)/1e3);
end

