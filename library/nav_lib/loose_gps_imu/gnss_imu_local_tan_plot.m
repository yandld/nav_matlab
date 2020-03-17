function gnss_imu_local_tan_plot(dataset, out_data, holdon)

%% GNSS position estimate','GNSS aided INS trajectory','Start point
h=zeros(1,3);
figure(2)
if ~holdon clf; end
plot(dataset.gnss.pos_ned(2,:),dataset.gnss.pos_ned(1,:),'b-');
hold on
h(1)=plot(dataset.gnss.pos_ned(2,:),dataset.gnss.pos_ned(1,:),'b.');
h(2)=plot(out_data.x(2,:),out_data.x(1,:),'r');
h(3)=plot(dataset.gnss.pos_ned(2,1),dataset.gnss.pos_ned(1,1),'ks');
title('Trajectory')
ylabel('North [m]')
xlabel('East [m]')
legend(h,'GNSS position estimate','GNSS aided INS trajectory','Start point')
axis equal
grid on

%% Height versus time
h=zeros(1,3);
figure(3)
if ~holdon clf; end
h(1)=plot(dataset.gnss.time,-dataset.gnss.pos_ned(3,:),'b.');
hold on
h(2)=plot(dataset.imu.time, -out_data.x(3,:),'r');
h(3)=plot(dataset.imu.time, 3*sqrt(out_data.diag_P(3,:)) - out_data.x(3,:),'k--');
plot(dataset.imu.time,-3*sqrt(out_data.diag_P(3,:))-out_data.x(3,:),'k--')
title('Height versus time')
ylabel('Height [m]')
xlabel('Time [s]')
grid on
legend(h,'GNSS estimate','GNSS aided INS estimate','3\sigma bound')

% figure(6)
% if ~holdon clf; end
% ylabels={'X-axis bias [m/s^2]','Y-axis bias [m/s^2]','Z-axis bias [m/s^2]'};
% h=zeros(1,2);
% for k=1:3
%     subplot(3,1,k)
%     h(1)=plot(dataset.imu.time,out_data.delta_u_h(k,:),'r');
%     hold on
%     h(2)=plot(dataset.imu.time,3*sqrt(out_data.diag_P(9+k,:))+out_data.delta_u_h(k,:),'k--');
%     plot(dataset.imu.time,-3*sqrt(out_data.diag_P(9+k,:))+out_data.delta_u_h(k,:),'k--')
%     grid on
%     ylabel(ylabels{k})
%     if k==1
%         title('Accelerometer bias estimate versus time')
%     end
% end
% xlabel('Time [s]')
% legend(h,'GNSS aided INS estimate','3\sigma bound')
% 
% 
% figure(7)
% if ~holdon clf; end
% ylabels={'X-axis bias [deg/s]','Y-axis bias [deg/s]','Z-axis bias [deg/s]'};
% h=zeros(1,2);
% for k=1:3
%     subplot(3,1,k)
%     h(1)=plot(dataset.imu.time,180/pi*out_data.delta_u_h(3+k,:),'r');
%     hold on
%     h(2)=plot(dataset.imu.time,3*180/pi*sqrt(out_data.diag_P(12+k,:))+180/pi*out_data.delta_u_h(3+k,:),'k--');
%     plot(dataset.imu.time,-3*180/pi*sqrt(out_data.diag_P(12+k,:))+180/pi*out_data.delta_u_h(3+k,:),'k--')
%     grid on
%     ylabel(ylabels{k})
%     if k==1
%         title('Gyroscope bias estimate versus time')
%     end
% end
% xlabel('Time [s]')
% legend(h,'GNSS aided INS estimate','3\sigma bound')



%% position difference
figure(8)
if ~holdon clf; end
xest = out_data.x(2,:);
yest = out_data.x(1,:);
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

