clear;
clc;
close all;
format short;

%%

rosbag info 'imu_error.bag'


bagselect  = rosbag('imu_error.bag');

 bagselect2 = select(bagselect,'Time',   [bagselect.StartTime bagselect.EndTime],'Topic', '/IMU_data_USB0');
 msgs = readMessages(bagselect2, "DataFormat", "struct");

N = length(msgs);
Fs = 100;

time = bagselect2.EndTime - bagselect2.StartTime;

fprintf('时间 %fs 共:%d帧, 帧率:%fHz\n', time, N, N / time);

accelReading=zeros(N,3);
gyroReading=zeros(N,3);
quat = zeros(N, 4);
eul = zeros(N, 3);

for i=1:N
    accelReading(i,1)=msgs{i,1}.LinearAcceleration.X;
    accelReading(i,2)=msgs{i,1}.LinearAcceleration.Y;
    accelReading(i,3)=msgs{i,1}.LinearAcceleration.Z;
    
    gyroReading(i,1)=msgs{i,1}.AngularVelocity.X;
    gyroReading(i,2)=msgs{i,1}.AngularVelocity.Y;
    gyroReading(i,3)=msgs{i,1}.AngularVelocity.Z;
    
    quat(i,1)=msgs{i,1}.Orientation.W;
    quat(i,2)=msgs{i,1}.Orientation.X;
    quat(i,3)=msgs{i,1}.Orientation.Y;
    quat(i,4)=msgs{i,1}.Orientation.Z;
    eul(i,:) = ch_q2eul(quat(i,:));
end

gyroReading = rad2deg(gyroReading);
eul = rad2deg(eul);

ch_plot_imu('time', (1:N)/Fs, 'acc', accelReading,  'gyr', gyroReading);


figure;
plot(eul(:,3))
title("yaw(deg)")

