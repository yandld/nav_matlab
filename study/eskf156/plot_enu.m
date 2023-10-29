function plot_enu(enu)
    %二维轨迹曲线
    plot(enu(:,1), enu(:,2), '.-'); hold on; axis equal; grid on; xlabel('East(m)'); ylabel('North(m)');  title('二维轨迹曲线');
end