%% Acc Calibraiton 

function [C, bias] = acc_calibration(data)

        %% Time- and Computation-Efficient Calibration of MEMS 3D Accelerometers and Gyroscopes
        % Data: M x N  M: sample set(6),  N:acc X,Y,Z (3)
        % row1 = 1, 0, 0 case
        % row2 = 0, 1, 0 case
        % row3 = 0, 0, 1 case
        % row4 = -1, 0, 0 case
        % row5 = 0, -1, 0 case
        % row6 = 0, 0, -1 case
        
   
            data = data';
            Asp = data(:,1:3);
            Asn = data(:,4:6);
            
            bias = ((Asp + Asn)*[1 1 1]' / 6);
            C = 2*(Asp - Asn)^(-1);
        end
        
        %% ST mehold AN4508 Application note Parameters and calibration of a low-g 3-axis accelerometer
%         function [C, bias]=mehold2(Data)
%             Y = [1 0 0; 0 1 0; 0 0 1; -1 0 0; 0 -1 0; 0 0 -1];
%             W = Data;
%             W= [W [1 1 1 1 1 1]'];
%             X = inv(W'*W) * W'*Y;
%             C = X(1:3,:)';
%             bias = -X(4,:)';
%         end