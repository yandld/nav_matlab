%% ACC Calibraiton

function [C, B] = acc_calibration(input)


%% OPTION1: ST mehold AN4508 Application note Parameters and calibration of a low-g 3-axis accelerometer
B = [1 0 0; 0 1 0; 0 0 1; -1 0 0; 0 -1 0; 0 0 -1];
A= [input -[ones(length(input), 1)]];
X = inv(A'*A) * A'*B;
C = X(1:3,:)';
B = X(4,:)';



%% OPTION2: Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
% Data: M x N  M: sample set(6),  N:acc X,Y,Z (3)
% row1 = 1, 0, 0 case
% row2 = 0, 1, 0 case
% row3 = 0, 0, 1 case
% row4 = -1, 0, 0 case
% row5 = 0, -1, 0 case
% row6 = 0, 0, -1 case


% input = input';
% Asp = input(:,1:3);
% Asn = input(:,4:6);
% 
% B = ((Asp + Asn)*[1 1 1]' / 6);
% C = 2*(Asp - Asn)^(-1);



end
