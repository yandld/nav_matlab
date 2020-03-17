%% gnss_imu_local_tan_example
clear;
clc;
close all;


%% Load data
disp('Loads data')


%%  Load Èðµädata
load('GNSSaidedINS_data.mat');
u = [in_data.IMU.acc; in_data.IMU.gyro];
gnss = in_data.GNSS.pos_ned;
gnss_time = in_data.GNSS.t;
imu_t = in_data.IMU.t;


%%  load couersera data
% load('p1_data.mat');
% in_data.IMU.acc = p1_data.imu_f.data';
% in_data.IMU.gyro = p1_data.imu_w.data';

% add noise
%in_data.IMU.acc = in_data.IMU.acc + rand(3, length(in_data.IMU.acc))*1;
 %in_data.IMU.gyro = in_data.IMU.gyro + deg2rad(  rand(3, length(in_data.IMU.gyro))*4 );
% in_data.IMU.gyro(3,:) = in_data.IMU.gyro(3,:)  + deg2rad(1);

% in_data.GNSS.pos_ned = p1_data.gnss.data';
% in_data.GNSS.t = p1_data.gnss.t;
% in_data.IMU.t = p1_data.imu_w.t';
% 
% u = [in_data.IMU.acc; in_data.IMU.gyro];
% gnss = in_data.GNSS.pos_ned;
% gnss_time = in_data.GNSS.t;
% imu_t = in_data.IMU.t;


%% load settings
settings = gnss_imu_local_tan_example_settings();

%% Run the GNSS-aided INS
disp('Runs the GNSS-aided INS')
cntr = 0;
x = init_navigation_state(u, settings);

% Initialize the sensor bias estimate
delta_u_h = zeros(3, 1);

% Initialize the Kalman filter
[P, Q, ~, ~] = init_filter(settings);

N = size(u,2);

ctr_gnss_data = 1;

for k=2:N
    
    % Sampling period
    dt = imu_t(k)-imu_t(k-1);
    
    % correction
    u_h = u(:,k);
    
    % nav_equ
    x = ch_nav_equ_local_tan(x, u_h, dt, settings.gravity);
    
   cntr = cntr+1;
   if cntr == 10
      
        %Get state space model matrices
        [F, G] = state_space_model(x, u_h, dt*cntr);
        
        %Time update of the Kalman filter state covariance.
        P = F*P*F' + G*Q*G';
        
       cntr = 0;
   end


    % measument update
    if abs(imu_t(k) - gnss_time(ctr_gnss_data)) < 0.01
        if imu_t(k)<settings.outagestart || imu_t(k) > settings.outagestop || ~strcmp(settings.gnss_outage,'on')
            
            y = gnss(:, ctr_gnss_data);
            H = [eye(3) zeros(3,6)];
            R = [settings.sigma_gps^2*eye(3)];
            
            % Calculate the Kalman filter gain.
             K=(P*H')/(H*P*H'+R);
            
             z =  K*(y - x(1:3));

            % Correct the navigation states using current perturbation estimates.
             x(1:6) = x(1:6)+z(1:6);
             
            %correct attitude
            q = x(7:10);
            q = ch_qmul(ch_qconj(q), ch_rv2q(z(7:9)));
            q = ch_qconj(q);
             x(7:10) = q;
             
           % delta_u_h = z(10:12);
            
            % Update the Kalman filter state covariance.
            P=(eye(9)-K*H)*P;
        end
        ctr_gnss_data = min(ctr_gnss_data+1, length(gnss_time));
    end

	% Save the data to the output data structure
	out_data.x(:,k) = x;
	out_data.diag_P(:,k) = diag(P);
	out_data.delta_u_h(:,k) = delta_u_h;
end

 gnss_imu_local_tan_plot(in_data, out_data, 'True');



%%  Init navigation state     %%
function x = init_navigation_state(u, settings)

roll = 0;
pitch = 0;

% Initial coordinate rotation matrix
q = ch_eul2q([roll pitch settings.init_heading]);
x = [zeros(6,1); q];

end

%%  Init filter  
function [P, Q, R, H] = init_filter(settings)


% Kalman filter state matrix
P = zeros(9);
P(1:3,1:3) = settings.factp(1)^2*eye(3);
P(4:6,4:6) = settings.factp(2)^2*eye(3);
P(7:9,7:9) = diag(settings.factp(3:5)).^2;

% Process noise covariance
Q = zeros(6);
Q(1:3,1:3) = diag(settings.sigma_acc).^2*eye(3);
Q(4:6,4:6) = diag(settings.sigma_gyro).^2*eye(3);


% GNSS-receiver position measurement noise
R = settings.sigma_gps^2*eye(3);

% Observation matrix
H = [eye(3) zeros(3,12)];

end


%%  State transition matrix
function [F,G] = state_space_model(x, u, t)
Cb2n = ch_q2m(x(7:10));

% Transform measured force to force in the tangent plane coordinate system.
sf = Cb2n * u(1:3);
St = ch_askew(sf);

% Only the standard errors included
O = zeros(3);
I = eye(3);
F = [ O I   O;
         O O St;
         O O O];

% Approximation of the discret
% time state transition matrix
F = eye(9) + t*F;

% Noise gain matrix
G=t*[O     O;
         Cb2n        O; 
         O       -Cb2n];
end



