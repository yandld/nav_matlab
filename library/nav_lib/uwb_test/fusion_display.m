%%  Author    : Gao Ouyang
%%  Date      : 2017.01.18
%%  Descriptor: 5 UWB anchor(only ranging) and 6-axis MEMS-IMU
%%              apply a kf demo for 3-D Location 
%%              states  : position,velocity,attitude,accel_bias,gyro_bias
%%              measures: ranging (unit: m)
%%              controls: accel , gyro(unit:deg/s)
%%

%-------------- figure 1: display trajectory ----------------------%
base = 1;

figure(10)
plot(TraceData(:,base+1), TraceData(:,base+2),  'g.')
hold on;
plot(StateByFilter(base,:), StateByFilter(base+1,:), 'm');
hold on;
plot(TrajectoryCollector(2,:), TrajectoryCollector(3,:), 'm');

title('Trajectory');xlabel('x:m');ylabel('y:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory', 'UWB');




figure(1)
subplot(311)
plot(TraceData(:,1),TraceData(:,base+1),'g.')
hold on
plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:),'m')
plot(TrajectoryCollector(1,:),TrajectoryCollector(2,:),'k')
title('Position x Axis');xlabel('T:s');ylabel('X axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')

subplot(312)
plot(TraceData(:,1),TraceData(:,base+2),'g.')
hold on
plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:),'m')
plot(TrajectoryCollector(1,:),TrajectoryCollector(3,:),'k')
title('Position y Axis');xlabel('T:s');ylabel('Y axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')

subplot(313)
plot(TraceData(:,1),TraceData(:,base+3),'g.')
hold on
plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:),'m')
plot(TrajectoryCollector(1,:),TrajectoryCollector(4,:),'k')
title('Position z Axis');xlabel('T:s');ylabel('Z axis:m');grid on;
legend('Real Trajectory','UWB-IMU Trajectory','UWB Trajectory')


% 
% %-------------- figure 2: display trajectory error -----------------%
% base = 1;
% figure(2)
% subplot(331)
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:)'- TraceData(:,base+1),'m');
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
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:)' - TraceData(:,base+2),'m')
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
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:)' - TraceData(:,base+3),'m')
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
% %-------------- figure 3: display Speed state  -----------------%
% base = 4;
% figure(3)
% subplot(311)
% plot(TraceData(:,1),TraceData(:,base+1),'r*');grid on
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:),'k')
% title('Speed x Axis');xlabel('T:s');ylabel('x axis:m');grid on;
% 
% subplot(312)
% plot(TraceData(:,1),TraceData(:,base+2),'g*');grid on
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:),'k')
% title('Speed y Axis');xlabel('T:s');ylabel('y axis:m');grid on;
% 
% subplot(313)
% plot(TraceData(:,1),TraceData(:,base+3),'c*');grid on
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:),'k')
% title('Speed z Axis');xlabel('T:s');ylabel('z axis:m');grid on;
% 
% %-------------- figure 4: display Pose state  -----------------%
% base = 7;
% figure(4)
% subplot(311)
% plot(TraceData(:,1),TraceData(:,base+1),'r*')
% hold on;grid on;
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:),'k')
% title('Euler');grid on;
% legend('Real Atti','UWB-IMU  Atti')
% 
% subplot(312)
% plot(TraceData(:,1),TraceData(:,base+2),'g*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:),'k')
% title('Euler');grid on;
% legend('Real Atti','UWB-IMU  Atti')
% 
% subplot(313)
% plot(TraceData(:,1),TraceData(:,base+3),'c*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:),'k')
% title('Euler');grid on;
% legend('Real Atti','UWB-IMU  Atti')
% 
% %-------------- figure 5: display estimated accel bias  ----------%
% base = 10;
% figure(5)
% subplot(311)
% plot(TraceData(:,1),TraceData(:,base+1),'r*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:),'k')
% title('Accel Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
% 
% subplot(312)
% plot(TraceData(:,1),TraceData(:,base+2),'g*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:),'k')
% title('Accel Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
% 
% subplot(313)
% plot(TraceData(:,1),TraceData(:,base+3),'c*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:),'k')
% title('Accel Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
% 
% %-------------- figure 6: display estimated gyro bias  ----------%
% base = 13;
% figure(6)
% subplot(311)
% plot(TraceData(:,1),TraceData(:,base+1),'r*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
% 
% subplot(312)
% plot(TraceData(:,1),TraceData(:,base+2),'g*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+1,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')
% 
% subplot(313)
% plot(TraceData(:,1),TraceData(:,base+3),'c*')
% hold on
% plot(SampleTimePoint(1:ImuPcs),StateByFilter(base+2,:),'k')
% title('Gyro Bias');grid on;
% legend('Real Error','UWB-IMU  Error')