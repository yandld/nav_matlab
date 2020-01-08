classdef fourati < handle
    methods (Static = true)
        
        %% Attitude Estimation for Indoor Navigation and Augmented Reality with Smartphones
        %%  https://github.com/tyrex-team/benchmarks-attitude-smartphones   QMichelObs.m

        function q = imu (q, Gyroscope, Accelerometer, SamplePeriod, Ka, Beta)
            
            % Normalise accelerometer measurement
            Measure = Accelerometer;
            if(norm(Measure) == 0), return; end   % handle NaN
            Measure = Measure / norm(Measure);    % normalise magnitude

            estimate_A  =  quatrotate(q, [0 0 1]);
            delta = (2 * Ka * skew_symmetric(estimate_A));

            K =  inv(delta' * delta + 1e-5 * eye(3)) * delta';
            qd = K * (Measure - estimate_A)';
            
            % integate 
            qDot = 0.5 * quatmultiply(q, [0 Gyroscope]) + Beta* quatmultiply(q, [0 qd']);
            
            q = q + qDot * SamplePeriod;
            q = q / norm(q); 
        end

    end
end


