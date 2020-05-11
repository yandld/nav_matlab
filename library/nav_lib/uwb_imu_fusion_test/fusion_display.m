function fusion_display(out_data, gt)

%% -------------- figure 1: display trajectory
fusion_pos = out_data.x(:,1:3);
fusion_speed = out_data.x(:,4:6);
fusion_quat = out_data.x(:,7:10);

%% plot uwb information
ch_plot_uwb(out_data.uwb, 2);

figure;
subplot(311)
hold on
plot(out_data.imu.time, fusion_pos(:,1),'m')
plot(length(out_data.uwb.pos), out_data.uwb.pos(:,1),'k')
title('Position x Axis');xlabel('T:s');ylabel('X axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')

subplot(312)
if( gt ~= [])
    plot(gt.time, gt.pos(:,2),'g.')
end
hold on
plot(out_data.imu.time, fusion_pos(:,2),'m')
plot(length(out_data.uwb.pos), out_data.uwb.pos(:,2),'k')
title('Position y Axis');xlabel('T:s');ylabel('Y axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')

subplot(313)
if( gt ~= [])
    plot(gt.time, gt.pos(:,3),'g.')
end
hold on
plot(out_data.imu.time, fusion_pos(:,3),'m')
plot(length(out_data.uwb.pos), out_data.uwb.pos(:,3),'k')
title('Position z Axis');xlabel('T:s');ylabel('Z axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')

%
% %-------------- figure 2: display trajectory error -----------------%
% base = 1;
% figure(2)
% subplot(331)
% plot(SampleTimePoint(1:N),StateByFilter(base,:)'- TraceData(:,base+1),'m');
% hold on
% plot(TrajectoryCollector(1,:),TrajectoryCollector(2,:) - TrajectoryCollector(5,:),'k')
% title('Position x Axis');xlabel('T:s');ylabel('X axis:m');grid on;
% legend('UWB-IMU Trajectory Error','UWB Trajectory Error')
%
% subplot(332)
% xvalues1 = -3:0.2:3;
% error = StateByFilter(base,:)'- TraceData(:,base+1);
% hist(error(find(error < 3 & error > -3)),100);
% title('Position x Axis Error Hist');grid on;
% legend('UWB-IMU Trajectory Error')
%
% h=subplot(333);
% error =TrajectoryCollector(2,:) - TrajectoryCollector(5,:);
% hist(error(find(error < 3 & error > -3)),100);
% hp = findobj(h,'Type','patch');
% set(hp,'FaceColor',[0 .5 .5],'EdgeColor','w')
% title('Position x Axis Error Hist');grid on;
% legend('UWB Trajectory Error')
%
% subplot(334)
% hold on
% plot(SampleTimePoint(1:N),StateByFilter(base+1,:)' - TraceData(:,base+2),'m')
% plot(TrajectoryCollector(1,:),TrajectoryCollector(3,:) - TrajectoryCollector(6,:),'k')
% title('Position y Axis');xlabel('T:s');ylabel('Y axis:m');grid on;
% legend('UWB-IMU Trajectory Error','UWB Trajectory Error')
%
% subplot(335)
% xvalues1 = -3:0.2:3;
% error = StateByFilter(base+1,:)'- TraceData(:,base+2);
% hist(error(find(error < 3 & error > -3)),100);
% title('Position x Axis Error Hist');grid on;
% legend('UWB-IMU Trajectory Error')
%
% h=subplot(336);
% error =TrajectoryCollector(3,:) - TrajectoryCollector(6,:);
% hist(error(find(error < 3 & error > -3)),100);
% hp = findobj(h,'Type','patch');
% set(hp,'FaceColor',[0 .5 .5],'EdgeColor','w')
% title('Position x Axis Error Hist');grid on;
% legend('UWB Trajectory Error')
%
% subplot(337)
% hold on
% plot(SampleTimePoint(1:N),StateByFilter(base+2,:)' - TraceData(:,base+3),'m')
% plot(TrajectoryCollector(1,:),TrajectoryCollector(4,:) - TrajectoryCollector(7,:),'k')
% title('Position z Axis');xlabel('T:s');ylabel('Z axis:m');grid on;
% legend('UWB-IMU Trajectory Error','UWB Trajectory Error')
%
% subplot(338)
% xvalues1 = -10:0.2:10;
% error = StateByFilter(base+2,:)'- TraceData(:,base+3);
% hist(error(find(error < 10 & error > -10)),100);
% title('Position x Axis Error Hist');grid on;
% legend('UWB-IMU Trajectory Error')
%
% h=subplot(339);
% error =TrajectoryCollector(4,:) - TrajectoryCollector(7,:);
% hist(error(find(error < 10 & error > -10)),100);
% hp = findobj(h,'Type','patch');
% set(hp,'FaceColor',[0 .5 .5],'EdgeColor','w')
% title('Position x Axis Error Hist');grid on;
% legend('UWB Trajectory Error')
%
%-------------- figure 3: display Speed state  -----------------%
% base = 4;
% figure(3)
% subplot(311)
% plot(gt.time, gt.speed(:,1),'r*');grid on
% hold on
% plot(SampleTimePoint(1:N),fusion_speed(:,1),'k')
% title('Speed x Axis');xlabel('T:s');ylabel('x axis:m');grid on;
%
% subplot(312)
% plot(gt.time,gt.speed(:,2),'g*');grid on
% hold on
% plot(SampleTimePoint(1:N), fusion_speed(:,2),'k')
% title('Speed y Axis');xlabel('T:s');ylabel('y axis:m');grid on;
%
% subplot(313)
% plot(gt.time, gt.speed(:,3),'c*');grid on
% hold on
% plot(SampleTimePoint(1:N), fusion_speed(:,3),'k')
% title('Speed z Axis');xlabel('T:s');ylabel('z axis:m');grid on;

%-------------- figure 4: display Pose state  -----------------%
eul = zeros(length(fusion_quat), 3);
for i = 1: length(fusion_quat)
    eul(i,:) = rad2deg(ch_q2eul(fusion_quat(i,:)));
end


figure;
subplot(311)
if( gt ~= [])
plot(gt.time, gt.eul(:,1),'r*')
end
hold on;grid on;
plot(out_data.imu.time, eul(:,1),'k')
title('Euler');grid on;
legend('Real Atti','UWB-IMU  Atti')


subplot(312)
if( gt ~= [])
plot(gt.time, gt.eul(:,2),'g*')
end
hold on
plot(out_data.imu.time, eul(:,2),'k')
title('Euler');grid on;
legend('Real Atti','UWB-IMU  Atti')

subplot(313)
if( gt ~= [])
plot(gt.time, gt.eul(:,3),'c*')
end
hold on
plot(out_data.imu.time, eul(:,3),'k')
title('Euler');grid on;
legend('Real Atti','UWB-IMU  Atti')

%-------------- figure 5: display estimated accel bias  ----------%
figure;
subplot(311)
if( gt ~= [])
plot(gt.time, gt.acc_bias(:,1),'r*')
end
hold on
plot(out_data.imu.time, out_data.delta_u(:,1),'k')
title('Accel Bias');grid on;
legend('Real Error','UWB-IMU  Error')

subplot(312)
if( gt ~= [])
plot(gt.time, gt.acc_bias(:,2),'g*')
end
hold on
plot(out_data.imu.time, out_data.delta_u(:,2),'k')
title('Accel Bias');grid on;
legend('Real Error','UWB-IMU  Error')

subplot(313)
if( gt ~= [])
plot(gt.time, gt.acc_bias(:,3),'c*')
end
hold on
plot(out_data.imu.time, out_data.delta_u(:,3),'k')
title('Accel Bias');grid on;
legend('Real Error','UWB-IMU  Error')

% %-------------- figure 6: display estimated gyro bias  ----------%
% base = 13;
% figure(6)
% subplot(311)
% plot(TraceData(:,1),TraceData(:,base+1),'r*')
% hold on
% plot(SampleTimePoint(1:N),StateByFilter(base,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
%
% subplot(312)
% plot(TraceData(:,1),TraceData(:,base+2),'g*')
% hold on
% plot(SampleTimePoint(1:N),StateByFilter(base+1,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
%
% subplot(313)
% plot(TraceData(:,1),TraceData(:,base+3),'c*')
% hold on
% plot(SampleTimePoint(1:N),StateByFilter(base+2,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')


end

