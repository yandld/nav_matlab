%% GYR Calibraiton
% example:
% input = [
%     65.2191    0.1712    0.4618
%    -0.0901   71.9079   -0.0728
%    -0.5425   -0.1436   71.7273
%   -65.0873   -0.1008   -0.4814
%     0.1573  -71.9432    0.0436
%     0.6128    0.2009  -71.7076
%     ];
% 
%  theory = [65.5618  0 0; 0 72.0649  0; 0 0 72.1298 ; -65.5081 0 0; 0 -72.1298 0; 0 0 -72.0649];
% gyr_calibration(input, theory)
function [C, B] = gyr_calibration(input, ref)

%%  ST mehold AN4508 Application note Parameters and calibration of a low-g 3-axis accelerometer
A= [input -[ones(length(input), 1)]];
B = ref;

X = inv(A'*A) * A'*B;
C = X(1:3,:)';
B = X(4,:)';


%% Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes 
% TBD

end

