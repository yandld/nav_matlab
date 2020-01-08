clear;
clc
close all;


%% INTPUS 
p = [0; 0];
v = [0;  0];
yaw = deg2rad(45);

%% Epoch 1 Measurements
omgea = [0.5; 0.2; 0.1; -0.1];
t = 0.5;
a = [2  0.1; 5 0.1; 2 -0.05; 0 0];

for i = 1:4
% update heading
lyaw = yaw;
yaw = yaw + omgea(i)*t;

% transform acceration to p frame, get a average value
Cpb_before = [cos(lyaw) -sin(lyaw); sin(lyaw) cos(lyaw)];
Cpb = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
Cpb = (Cpb_before + Cpb) / 2;

% get acceralation in N frame
a_n_frame = Cpb*a(i,:)';

% update velocity
lv = v;
v = v + a_n_frame.*t;
% update position
p = p + (lv +v)*0.5*t;

end

p
