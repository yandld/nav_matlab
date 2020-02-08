%%  Author    : Gao Ouyang
%%  Date      : 2017.01.18
%%  Descriptor: 5 UWB anchor(only ranging) and 6-axis MEMS-IMU
%%              apply a ukf demo for 3-D Location, please see ReadMe.docx in detail. 
%%              states  : position,velocity,attitude,accel_bias,gyro_bias
%%              measures: ranging (unit: m)
%%              controls: accel , gyro(unit:deg/s)
%%

run fusion_init.m

%% Motion Process, Measurement model and it's derivative
f_func = @ukf_ins_f;
h_func = @uwb_h;

imu_iter = 1;
uwb_iter = 1;
dt = 0.02;
ImuPcs = length(SampleTimePoint);
ISPUTCHAR = 0;
% Reserve space for estimates.
StateByFilter = zeros(size(X,1),ImuPcs);
StateCoVariance = zeros(size(X,1),size(X,1),ImuPcs);
Invation = zeros(5,ImuPcs);
TrajectoryCollector= []; noise=[];
% Estimate with UKF
for k=1:ImuPcs
	uk = [a_vector(imu_iter,:)'; g_vector(imu_iter,:)'/180*pi];
	
    % Track with (non-augmented) UKF
    [X,P] = ukf_predict1(X,P,f_func,Q, [dt;uk]);

	if ISPUTCHAR == 1
		cprintf('text', 'time: %8.3f s, Position = [%0.2f %0.2f %0.2f] m, Velocity = [%0.3f %0.3f %0.3f] m/s Position Variance = [%0.5f %0.5f %0.5f], Velocity Variance = [%0.5f %0.5f %0.5f]m/s^2\n',...
				SampleTimePoint(imu_iter) ,X(1),X(2),X(3),X(4),X(5),X(6), P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6)); 
	end
	
	%%% Update
	if uwb_iter <= length(UWBBroadTime_vector) && UWBBroadTime_vector(uwb_iter)== SampleTimePoint(imu_iter) 	
		Z_meas = Uwbranging_vector(uwb_iter,:)';
		
		%%%add the noise
		%outlier = zero_one_distribution(0.1)';
        outlier = zeros(5,1); %% you could ignore annotation without outlier
		Z_meas = Z_meas + outlier;
		noise = [noise, outlier];
			
		[X,P] = ukf_update1(X,P,Z_meas,h_func,R); 
        P = (P + P')/2;
		if ISPUTCHAR == 1
			cprintf('err', 'time: %8.3f s, Position [%0.2f %0.2f %0.2f] m, Velocity [%0.3f %0.3f %0.3f] m/s Position Variance = [%0.5f %0.5f %0.5f], Velocity Variance = [%0.5f %0.5f %0.5f]m/s^2\n\n\n',...
				    UWBBroadTime_vector(uwb_iter) ,X(1),X(2),X(3),X(4),X(5),X(6), P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6)); 
		end
		uwb_iter = uwb_iter + 1;
   end
	   
   StateByFilter(:,k)   = X;
   StateCoVariance(:,:,k) = P;
   imu_iter = imu_iter + 1;
end  

StateByFilter(7:9,:)= StateByFilter(7:9,:)/pi*180;
noise = noise';
for uwb_iter=1:4:length(UWBBroadTime_vector)-10
    Z_meas = diag(Uwbranging_vector(uwb_iter:uwb_iter+4,:) +  noise(uwb_iter:uwb_iter+4,:)) ;
    uwbxyz = triangulate(Z_meas);
    TrajectoryCollector= [TrajectoryCollector,[UWBBroadTime_vector(uwb_iter+2); uwbxyz; TraceData(4*(uwb_iter+2)+1,2:4)']];
end

run fusion_display.m