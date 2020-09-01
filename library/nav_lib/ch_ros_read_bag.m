%% Example
% path = 'imu3_codeodom_tst.bag';
% 
% rosbag info 'imu3_codeodom_tst.bag'
% [imu, eul]=  ch_ros_read_bag(path,  '/imu_hi226');
% ch_imu_data_plot('acc', imu.acc',  'gyr', imu.gyr', 'eul', eul',  'time',  imu.time', 'subplot', 1);
%%


function [imu, eul]= ch_ros_read_bag(path, imu_topic_name)

bag = rosbag(path);

%œ‘ æbag–≈œ¢
% rosbag info 'imu3_codeodom_tst.bag'

% bag = select(bag, 'Time', [bag.StartTime+100  bag.StartTime + 110], 'Topic',imu_topic_name);
bag = select(bag, 'Topic', imu_topic_name);


imu_cell = readMessages(bag);

n = length(imu_cell);

span = bag.EndTime -  bag.StartTime ;
imu.time = 0: span / n : span;
imu.time(end) = []; % remove last element

for i = 1:n
    imu.gyr(1,i) = imu_cell{i,1}.AngularVelocity.X;
    imu.gyr(2,i) = imu_cell{i,1}.AngularVelocity.Y;
    imu.gyr(3,i) = imu_cell{i,1}.AngularVelocity.Z;
    
    imu.acc(1,i) = imu_cell{i,1}.LinearAcceleration.X;
    imu.acc(2,i) = imu_cell{i,1}.LinearAcceleration.Y;
    imu.acc(3,i) = imu_cell{i,1}.LinearAcceleration.Z;
    
    q(1) =  imu_cell{i, 1}.Orientation.W;
    q(2) =  imu_cell{i, 1}.Orientation.X;
    q(3) =  imu_cell{i, 1}.Orientation.Y;
    q(4) =  imu_cell{i, 1}.Orientation.Z;
    eul(:, i) = ch_q2eul(q);
end

eul = rad2deg(eul);

end