function eul = ch_q2eul(q)
%q2eul - Converts a quatnrion to the
%corresponding set of Euler angles%
%
%
% Inputs:
%   q       coordinate transformation matrix describing transformation from
%           beta(N) to alpha(B)
%
% Outputs:
%   eul     Euler angles describing rotation from beta to alpha in the 
%           order roll, pitch, yaw(rad)


% Begins

eul(1,1) = atan2(2 * ( q(1)*q(2) + q(3)*q(4) ) , 1 - 2*q(2)*q(2) - 2*q(3)*q(3));  % roll
eul(2,1) = asin(2*(q(1)*q(3) - q(2)*q(4)) );        % pitch
eul(3,1) = atan2(2 * ( q(1)*q(4) + q(2)*q(3) ), 1 - 2*q(3)*q(3) - 2*q(4)*q(4));  % yaw

% Ends