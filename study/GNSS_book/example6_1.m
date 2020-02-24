clear;
clc
close all;

addpath('../../library/nav_lib'); 

%% attidude

true_attitude = deg2rad([4 -6 30])';
roll_meas_err = deg2rad(0.1);
pitch_meas_err = deg2rad(-0.15);

%% Earth's Magnetic Field
flux = 45;
declination= deg2rad(10);
inclination = deg2rad(40);

delta_alpha_nE= deg2rad(0.3);

%% Other Souces of Magnetism
MAn = [2 -1.8 3]'; % local magnetic anomalies(N,E,D)
hard = [-1 -0.5 1]'; % hard-iron magnetism(body-frame axes)
soft = [0.012 -0.014 0.006; -0.008 0.007 0.017; 0.003 -0.019 -0.011]; % soft-iron magnetism, body-rame-axes


%% 
Cn2b = ch_eul2m(true_attitude);

%% flux desity of Earth's magnetic filed
m_En = [cos(declination)*cos(inclination)  sin(declination)*cos(inclination) sin(inclination)]'*flux;

%% Total magnetic flux density
m_mb = hard + (eye(3) + soft)*Cn2b*(m_En + MAn);

%% Calculate Magnetic Heading Measurement
roll_meas = true_attitude(1) + roll_meas_err;
pitch_meas =  true_attitude(2) +pitch_meas_err;

yaw = atan2(-m_mb(2)*cos(roll_meas) + m_mb(3)*sin(roll_meas), m_mb(1)*cos(pitch_meas) + m_mb(2)*sin(pitch_meas)*sin(roll_meas) + m_mb(3)*cos(roll_meas)*sin(pitch_meas));


%% Calculate True Heading
data_base_magnetic_declination = declination + delta_alpha_nE

%% True heading
true_heading = yaw+ data_base_magnetic_declination;
%rad2deg(true_heading)

%% Heading error
error = true_heading - true_attitude(3);
rad2deg(error)
