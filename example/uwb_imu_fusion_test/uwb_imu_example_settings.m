
function settings = gnss_imu_local_tan_example_settings()


settings.sigma_acc = 0.2;   %加速度计噪声
settings.sigma_gyro = deg2rad(0.5);  %陀螺仪噪声
settings.sigma_acc_bias = 0.00;  %加速度计零偏随机游走噪声 
settings.sigma_gyro_bias = deg2rad(0.0);  %陀螺仪零偏随机游走噪声
 


% Initial Kalman filter uncertainties (standard deviations)
settings.factp(1) = 100;                                 % Position [m]
settings.factp(2) = 10;                                   % Velocity [m/s]
settings.factp(3:5) = deg2rad([100 100 100]);     % Attitude (roll,pitch,yaw) [rad]
settings.factp(6) = 0.03;                               % Accelerometer biases [m/s^2]
settings.factp(7) = deg2rad(0.05);                     % Gyro biases [rad/s]


end




