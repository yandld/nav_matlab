function eul = ch_q2eul(q, seq)
% ch_q2eul: transforms quaternion to Euler angles.
%
% INPUT
%   qin: 4x1 quaternion.
%
% OUTPUT
%   euler: 3x1 Euler angles [roll pitch yaw] (rad, rad, rad).

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

if nargin == 2
    
    if  strcmp(seq,'312')
        roll =  -atan2( 2*( q1*q3 - q0*q2 ) , q0*q0 - q1*q1 - q2*q2 + q3*q3);
        pitch = asin( 2*(q0*q1 + q2*q3) );
        yaw = -atan2(2*( q1*q2 - q0*q3 ), q0*q0 - q1*q1 + q2*q2 - q3*q3);
    elseif strcmp(seq,'321')
        roll =  atan2( 2*( q0*q1 + q2*q3 ) , 1 - 2*q1*q1 - 2*q2*q2);
        pitch = asin( 2*(q0*q2 - q1*q3) );
        yaw = atan2(2*( q0*q3 + q1*q2 ), 1 - 2*q2*q2 - 2*q3*q3);
        else
        error("un-supported eular sequence");
    end
else
    % 321 sequence
    roll =  atan2( 2*( q0*q1 + q2*q3 ) , 1 - 2*q1*q1 - 2*q2*q2);
    pitch = asin( 2*(q0*q2 - q1*q3) );
    yaw = atan2(2*( q0*q3 + q1*q2 ), 1 - 2*q2*q2 - 2*q3*q3);
end


eul(1,1) = roll;
eul(2,1) = pitch;
eul(3,1) = yaw;




