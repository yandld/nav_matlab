% subplot£ºÊÇ·ñ¿ªÆôsubplot
function ch_plot_gps_imu_pos(varargin)
%%  plot imu data
i = 1;
param= inputParser;
param.addOptional('time', []);
param.addOptional('pos', []);
param.addOptional('gnss', []);

param.parse(varargin{:});
r = param.Results;

if(r.time == 0 )
    error('no time data');
end

figure;
subplot(2,1,1);
plot(r.gnss(:,2), r.gnss(:,1),'b-');
hold on;
plot(r.gnss(:,2), r.gnss(:,1),'b.');
plot(r.pos(:,2), r.pos(:,1), 'r-');
plot(r.pos(1,1), r.pos(1,2),'ks');
legend('GNSS position estimate','GNSS aided INS trajectory','Start point')
axis equal
hold off;
xlabel('X(m)'); ylabel('Y(m)');  title('Trajectory');

subplot(2,1,2);
hold on;
plot(1:length(r.gnss), -r.gnss(:,3),'b.');
plot(r.time, -r.pos(:,3),'r');
legend('GNSS estimate','GNSS aided INS estimate')
title('Height versus time');  xlabel('Time [s]');  ylabel('Height [m]');
hold off;


end

