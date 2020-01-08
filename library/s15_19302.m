classdef s15_19302 < handle
    methods (Static = true)
        
        function qo = UpdateIMU(qi, Gyroscope, Accelerometer, SamplePeriod, acc_gain)
            
            % Normalise accelerometer measurement
            norm_acc =  norm(Accelerometer);
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            
            gN = quatrotate(quatconj(qi), Accelerometer);
            
            delta(1) = -sqrt((gN(3) + 1) / 2);
            delta(2) = -gN(2) / sqrt(2*(gN(3) + 1));
            delta(3) =  gN(1) / sqrt(2*(gN(3) + 1));
            delta(4) = 0;
            
            if abs(norm_acc -1.0)<0.02
            else
                acc_gain = 0;
            end
            
            delta = quatinterp([1 0 0 0], delta , acc_gain,'slerp');
            qi = quatmultiply(qi, delta);
            
            % integate
            dq = quatmultiply(qi, [0 Gyroscope] * SamplePeriod/2);
            qo = qi + dq;
            qo =  qo / norm(qo);
        end
        
        function qo = UpdateAHRS(qi, Gyroscope, Accelerometer, Magnetometer, SamplePeriod, acc_gain, mag_gain)
            
            % Normalise accelerometer measurement
            norm_acc =  norm(Accelerometer);
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            
            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            % acc correction
            gN = quatrotate(quatconj(qi), Accelerometer);
            
            delta(1) = -sqrt((gN(3) + 1) / 2);
            delta(2) = -gN(2) / sqrt(2*(gN(3) + 1));
            delta(3) =  gN(1) / sqrt(2*(gN(3) + 1));
            delta(4) = 0;
            
            delta = quatinterp([1 0 0 0], delta ,acc_gain,'slerp');
            
            if abs(norm_acc -1.0)<0.2
                qi = quatmultiply(qi, delta);
            end
            
            % mag correction
            mN = quatrotate(quatconj(qi), Magnetometer);
            GAMMA = (mN(1)^2 + mN(2)^2);
            
            delta(1) = -sqrt(GAMMA + mN(1)*sqrt(GAMMA)) / sqrt(2*GAMMA);
            delta(2) = 0;
            delta(3) = 0;
            delta(4) = mN(2) / (sqrt(2*(GAMMA + mN(1)*sqrt(GAMMA))));
            
            delta = quatinterp([1 0 0 0], delta ,mag_gain,'slerp');
            
            qi = quatmultiply(qi, delta);
            
            % integate
            dq = quatmultiply(qi, [0 Gyroscope] * SamplePeriod/2);
            qo = qi + dq;
            qo =  qo / norm(qo);
        end
        
    end
end