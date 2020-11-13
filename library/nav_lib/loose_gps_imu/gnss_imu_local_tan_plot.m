function gnss_imu_local_tan_plot(dataset, outdata, holdon)

%% GNSS position estimate','GNSS aided INS trajectory','Start point
% h=zeros(1,3);
% figure(2)
% if ~holdon clf; end
% plot(dataset.gnss.pos_ned(2,:),dataset.gnss.pos_ned(1,:),'b-');
% hold on
% h(1)=plot(dataset.gnss.pos_ned(2,:),dataset.gnss.pos_ned(1,:),'b.');
% h(2)=plot(outdata.x(2,:),outdata.x(1,:),'r');
% h(3)=plot(dataset.gnss.pos_ned(2,1),dataset.gnss.pos_ned(1,1),'ks');
% title('Trajectory')
% ylabel('North [m]')
% xlabel('East [m]')
% legend(h,'GNSS position estimate','GNSS aided INS trajectory','Start point')
% axis equal
% grid on
% 
% %% Height versus time
% h=zeros(1,3);
% figure(3)
% if ~holdon clf; end
% h(1)=plot(dataset.gnss.time,-dataset.gnss.pos_ned(3,:),'b.');
% hold on
% h(2)=plot(dataset.imu.time, -outdata.x(3,:),'r');
% h(3)=plot(dataset.imu.time, 3*sqrt(outdata.diag_P(3,:)) - outdata.x(3,:),'k--');
% plot(dataset.imu.time,-3*sqrt(outdata.diag_P(3,:))-outdata.x(3,:),'k--')
% title('Height versus time')
% ylabel('Height [m]')
% xlabel('Time [s]')
% grid on
% legend(h,'GNSS estimate','GNSS aided INS estimate','3\sigma bound')


%% position difference
figure(8)
if ~holdon clf; end
xest = outdata.x(2,:);
yest = outdata.x(1,:);
xgps = interp1(dataset.gnss.time, dataset.gnss.pos_ned(2,:), dataset.imu.time,'linear','extrap')';
ygps = interp1(dataset.gnss.time, dataset.gnss.pos_ned(1,:), dataset.imu.time,'linear','extrap')';
xerr = xest - xgps; 
yerr = yest - ygps;
plot(dataset.imu.time, xerr)
grid on
hold on
plot(dataset.imu.time, yerr)
xlabel('time [s]')
ylabel('position difference [m]')
legend('x', 'y')

positionerr_RMS = sqrt(mean(xerr.^2+yerr.^2))
figure(2)

end

