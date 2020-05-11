%% Accelerometer Calibration Example

%% start
clc;
clear; 
close all;

%% sample data  for acc cal

 data(1,:) =  [  1.0556   0.0194  -0.0651  ];
 data(2,:) = [ 0.0195   0.9952  -0.0430 ];
 data(3,:) = [  0.0714  -0.0175   0.9652  ];
 data(4,:) =  [  -0.9272  -0.0181  -0.0296 ];
 data(5,:) =  [   0.0921  -0.9974  -0.0491 ];
 data(6,:) =  [  0.0503   0.0159  -1.0561 ];
 
%% Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
	[C, bias] = acc_calibration(data);

	res(1,:) = C*(data(1,:)' - bias); 
	res(2,:) = C*(data(2,:)' - bias); 
	res(3,:) = C*(data(3,:)' - bias); 

	res(4,:) = C*(data(4,:)' - bias); 
	res(5,:) = C*(data(5,:)' - bias); 
	res(6,:) = C*(data(6,:)' - bias); 

%% ST mehold
% AN4508
%Application note
%Parameters and calibration of a low-g 3-axis accelerometer

% 	[C, bias] = acc_cal.mehold2(Data);
% 
% 	Result(1,:) = C*(Data(1,:)' - bias); 
% 	Result(2,:) = C*(Data(2,:)' - bias); 
% 	Result(3,:) = C*(Data(3,:)' - bias); 
% 
% 	Result(4,:) = C*(Data(4,:)' - bias); 
% 	Result(5,:) = C*(Data(5,:)' - bias); 
% 	Result(6,:) = C*(Data(6,:)' - bias);
%     
%     C
%     bias
%     
% hold on; 
% grid on;
 plot3(data(:,1), data(:,2), data(:,3), 'or'); 
 hold on;
 plot3(res(:,1), res(:,2), res(:,3), '*b'); 
 
 legend('输入', '校准后');
