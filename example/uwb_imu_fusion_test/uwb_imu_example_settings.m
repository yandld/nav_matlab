
function settings = uwb_imu_example_settings()


settings.sigma_acc = 0.01;                   %加速度计噪声
settings.sigma_gyro = deg2rad(0.3);      %陀螺仪噪声
settings.sigma_acc_bias = 0.00;              %加速度计零偏随机游走噪声 
settings.sigma_gyro_bias = deg2rad(0.0);  %陀螺仪零偏随机游走噪声
 


% Initial Kalman filter uncertainties (standard deviations)
settings.factp(1) = 1;                                 % 初值位置方差(置信度)(m)，越大代表对初始位置越不信任
settings.factp(2) = 1;                                   % 初值速度方差( [m/s]
settings.factp(3:5) = deg2rad([1 1 20]);     % 初始i姿态方差 (roll,pitch,yaw) [rad]
settings.factp(6) = 0.03;                               % 初始加速度零偏方差 [m/s^2]
settings.factp(7) = deg2rad(0.2);                     % 初始角速度零偏方差 [rad/s]


end




