classdef fcf < handle
    methods (Static = true)
       % Fast Complementary Filter for Attitude Estimation Using Low-Cost
       % MARG Sensors  电子科大 小吴

        
        function q = imu(q, Gyroscope, Accelerometer, SamplePeriod, Beta)
            
            % Normalise accelerometer measurement
            acc = Accelerometer;
            if(norm(acc) == 0), return; end   % handle NaN
            acc = acc / norm(acc);  % normalise magnitude
            
            gyr = Gyroscope;
           F = [0 -gyr(1) -gyr(2) -gyr(3); gyr(1) 0 gyr(3) -gyr(2); gyr(2) -gyr(3) 0 gyr(1); gyr(3) gyr(2) -gyr(1) 0];
           Wa = [acc(3) acc(2) -acc(1) 0; acc(2) -acc(3) 0 acc(1); -acc(1) 0 -acc(3) acc(2); 0 acc(1) acc(2) acc(3)];
           
           q =((eye(4) + ((1 - Beta) * SamplePeriod / 2) * F)  + Beta /2 * (Wa - eye(4))  ) * q';
          
            q = q';
            q = q / norm(q); % normalise quaternion
        end
    end
end