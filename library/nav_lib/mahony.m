classdef mahony < handle
    methods (Static = true)
        % mahony IMU modifiy by OMID
        function q = imu(q, Gyroscope, Accelerometer, SamplePeriod, Kp)
            
            % Normalise accelerometer measurement
            norm_acc =  norm(Accelerometer);
            if((norm_acc) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            
            % OMID  modifcation 
            qtmp = q + 0.5 * qmul(q, [0 Gyroscope] * SamplePeriod)';
            
            % Error is sum of cross product between estimated direction and measured direction of fields
            v = quatrotate(qtmp, [0 0 1]);
           % v = quatrotate(q, [0 0 1]);
           
            e = cross(Accelerometer, v) ;
            
            % Apply feedback terms
            Gyroscope = Gyroscope + Kp * e ;
            
            % integate
            q = q + 0.5 * qmul(q, [0 Gyroscope] * SamplePeriod)';
            q =  q / norm(q);
        end

        
        function q = ahrs(q, Gyroscope, Accelerometer, Magnetometer, SamplePeriod, Kp)
            
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            % Reference direction of Earth's magnetic feild
            h = quatrotate(quatconj(q), Magnetometer);
            b = [norm([h(1) h(2)]) 0 h(3)];
            
            % Estimated direction of gravity and magnetic field
            w = quatrotate(q, b);
            v = quatrotate(q, [0 0 1]);
            
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v) + cross(Magnetometer, w);
            
            % Apply feedback terms
            Gyroscope = Gyroscope + Kp * e ;
            
            % integate
            dq = qmul(q, [0 Gyroscope] * SamplePeriod/2);
            q = q + dq;
            q =  q / norm(q);
        end
    end
end