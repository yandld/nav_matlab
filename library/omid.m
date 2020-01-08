classdef omid < handle
    methods (Static = true)
        
        % OMID is a little modification of Mad filter, see:  On the Accuracy Improvement of Low-Power Orientation 1 Filters Using IMU and MARG Sensor Arrays

        function q = imu (q, Gyroscope, Accelerometer, SamplePeriod, Beta)
            
            % Normalise accelerometer measurement
            acc = Accelerometer;
            if(norm(acc) == 0), return; end   % handle NaN
            acc = acc / norm(acc);    % normalise magnitude
            
            % integate 
            q = q + 0.5 * quatmultiply(q, [0 Gyroscope]) * SamplePeriod;
            
            % Gradient decent algorithm corrective step
            F = (quatrotate(q, [0 0 1]) - acc)';

            J = [-2*q(3),	2*q(4),    -2*q(1),	  2*q(2)
                   2*q(2),    2*q(1),     2*q(4),    2*q(3)
                   0,           -4*q(2),   -4*q(3),	        0];
            step = (J'*F);
            
            % update q
            q = q  - Beta * step' * SamplePeriod;
            
            % normalise quaternion
            q = q / norm(q); 
        end

    end
end