classdef madgwick < handle
    methods (Static = true)
        
        function q = imu(q, Gyroscope, Accelerometer, SamplePeriod, Beta)
            
            % Normalise accelerometer measurement
            acc = Accelerometer;
            if(norm(acc) == 0), return; end   % handle NaN
            acc = acc / norm(acc);  % normalise magnitude
            
            % Gradient decent algorithm corrective step
            F = (quatrotate(q, [0 0 1]) - acc)';
%             F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
%                 2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
%                 2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];

            J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
                2*q(2),     2*q(1),     2*q(4),	2*q(3)
                0,         -4*q(2),    -4*q(3),	0    ];
            step = (J'*F);
            step = step / norm(step);	% normalise step magnitude
            
            % Compute rate of change of quaternion
            qd = 0.5 * quatmultiply(q, [0 Gyroscope]) - Beta * step';
            
            % Integrate to yield quaternion
            q = q + qd * SamplePeriod;
            q = q / norm(q); % normalise quaternion
        end
        
        function q = ahrs(q, Gyroscope, Accelerometer, Magnetometer, SamplePeriod, Beta)
            
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            acc = Accelerometer / norm(Accelerometer);	% normalise magnitude
            
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end	% handle NaN
            mag = Magnetometer / norm(Magnetometer);	% normalise magnitude
            
            % Reference direction of Earth's magnetic feild
            h = quatrotate(quatconj(q), mag);
            h = [0 h];
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            
            % Gradient decent algorithm corrective step
             F= (quatrotate(q, [0 0 1]) - acc)';
             F = [F;  (quatrotate(q, [b(2) 0 b(4)]) - mag)'];
            
%             F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
%                 2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
%                 2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
%                 2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
%                 2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
%                 2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
            J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
                2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
                0,                         -4*q(2),                    -4*q(3),                         0
                -2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
            step = (J'*F);
            step = step / norm(step);	% normalise step magnitude
            
            % Compute rate of change of quaternion
            qd = 0.5 * quatmultiply(q, [0 Gyroscope]) - Beta * step';

            % Integrate to yield quaternion
            q = q + qd * SamplePeriod;
            q = q / norm(q); % normalise quaternion
        end
    end
end