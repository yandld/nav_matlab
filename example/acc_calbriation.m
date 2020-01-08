%% Accelerometer Calibration Example

%% start
clc; clear; close all;
addpath('../library');

%% sample data  for acc cal
 Data(1,:) = [  1.0126   0.0133  -0.0351]; % 1 0 0 case 
 Data(2,:) = [  0.0304   1.0079  -0.0409];
 Data(3,:) = [  0.0403   0.0394   0.9865];
 
 Data(4,:) = [ -0.9715   0.0305   0.0154];
 Data(5,:) = [  0.0234  -0.9759   0.0110];
 Data(6,:) = [  0.0098   0.0152  -1.0087]; %0 0 -1 case

  
  Data(1,:) =  [  1.0556   0.0194  -0.0651  ];
  Data(2,:) = [ 0.0195   0.9952  -0.0430 ];
  Data(3,:) = [  0.0714  -0.0175   0.9652  ];
 Data(4,:) =  [  -0.9272  -0.0181  -0.0296 ];
 Data(5,:) =  [   0.0921  -0.9974  -0.0491 ];
 Data(6,:) =  [  0.0503   0.0159  -1.0561 ];
 
%% Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
	[C, bias] = acc_cal.mehold1(Data);

	Result(1,:) = C*(Data(1,:)' - bias); 
	Result(2,:) = C*(Data(2,:)' - bias); 
	Result(3,:) = C*(Data(3,:)' - bias); 

	Result(4,:) = C*(Data(4,:)' - bias); 
	Result(5,:) = C*(Data(5,:)' - bias); 
	Result(6,:) = C*(Data(6,:)' - bias); 

%% ST mehold
% AN4508
%Application note
%Parameters and calibration of a low-g 3-axis accelerometer

	[C, bias] = acc_cal.mehold2(Data);

	Result(1,:) = C*(Data(1,:)' - bias); 
	Result(2,:) = C*(Data(2,:)' - bias); 
	Result(3,:) = C*(Data(3,:)' - bias); 

	Result(4,:) = C*(Data(4,:)' - bias); 
	Result(5,:) = C*(Data(5,:)' - bias); 
	Result(6,:) = C*(Data(6,:)' - bias);
    
    C
    bias
    
hold on; 
grid on;
plot3(Data(:,1), Data(:,2), Data(:,3), 'or'); plot3(Result(:,1), Result(:,2), Result(:,3), '*b'); 
legend('输入', '校准后');

%% 参考数据 
% 输入：
%  Data(1,:) = [  1.0126   0.0133  -0.0351];
%  Data(2,:) = [  0.0304   1.0079  -0.0409];
%  Data(3,:) = [  0.0403   0.0394   0.9865];
%  
%  Data(4,:) = [ -0.9715   0.0305   0.0154];
%  Data(5,:) = [  0.0234  -0.9759   0.0110];
%  Data(6,:) = [  0.0098   0.0152  -1.0087];
% 
%  输出：mehold1 Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
%      0.9963    0.0001    0.0020
%     0.0028    0.9943   -0.0031
%     0.0009    0.0056    1.0010
%    -1.0037    0.0001    0.0020
%     0.0028   -1.0057   -0.0031
%     0.0009    0.0056   -0.9990
%  输出：mehold2
%     0.9960   -0.0004    0.0009
%     0.0026    0.9937   -0.0043
%     0.0006    0.0051    0.9998
%    -1.0039   -0.0004    0.0008
%     0.0025   -1.0062   -0.0042
%     0.0006    0.0051   -1.0001

