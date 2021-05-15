classdef ch_mahony < handle
    methods (Static = true)
        function q = imu(q, gyr, acc, dt, Kp)
            
            % 加速度单位化
            norm_acc =  norm(acc);
            if((norm_acc) == 0), return; end 
            acc = acc / norm(acc);
            
            qtmp = ch_att_upt(q, gyr, dt);
            
            v = ch_qmulv(qconj(qtmp), [0 0 1]');
            
            e = cross(acc, v) ;
            
            % 反馈
            gyr = gyr + Kp * e ;
            
            % 积分
            q = ch_qintg(q, gyr, dt);
        end
        
        
        function q = ahrs(q, gyr, acc, mag, dt, Kp)
            
            % 加速度计单位化
            if(norm(acc) == 0), return; end   % handle NaN
            acc = acc / norm(acc);    % normalise magnitude
            
            % 磁场单位化
            if(norm(mag) == 0), return; end    % handle NaN
            mag = mag / norm(mag);   % normalise magnitude
            
            % Reference direction of Earth's magnetic feild
            h = ch_qmulv(q, mag);
            b = [norm([h(1) h(2)]) 0 h(3)]';
            
            % Estimated direction of gravity and magnetic field
            w = ch_qmulv(ch_qconj(q), b);
            v = ch_qmulv(ch_qconj(q), [0 0 1]');
            
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(acc, v) + cross(mag, w);
            
            % Apply feedback terms
            gyr = gyr + Kp * e ;
            
            % integate
            q = ch_qintg(q, gyr, dt);
        end
    end
end