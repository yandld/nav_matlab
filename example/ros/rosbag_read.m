clear;
clc;
close all;
format short;

%%

%rosbag info 'T0_raw_mopping0.bag'

bagselect  = rosbag('T1_raw_sweeping0.bag');

bagselect2 = select(bagselect,'Time',   [bagselect.StartTime bagselect.EndTime],'Topic', '/mobile_base/sensors/pose3D');
msgs = readMessages(bagselect2, "DataFormat", "struct");

N = length(msgs);
Fs = 100;

time = bagselect2.EndTime - bagselect2.StartTime;

fprintf('时间 %fs 共:%d帧, 帧率:%fHz\n', time, N, N / time);

accelReading=zeros(N,3);
gyroReading=zeros(N,3);
for i=1:N
    accelReading(i,1)=msgs{i,1}.AccelerationX;
    accelReading(i,2)=msgs{i,1}.AccelerationY;
    accelReading(i,3)=msgs{i,1}.AccelerationZ;
    
    gyroReading(i,1)=msgs{i,1}.OmegaX;
    gyroReading(i,2)=msgs{i,1}.OmegaY;
    gyroReading(i,3)=msgs{i,1}.OmegaZ;
end

gyroReading = rad2deg(gyroReading);

ch_imu_data_plot('time', (1:N)/Fs, 'acc', accelReading);
ch_imu_data_plot('time', (1:N)/Fs, 'gyr', gyroReading);

