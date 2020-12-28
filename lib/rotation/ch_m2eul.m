function eul = ch_m2eul(Cb2n, seq)
% ch_m2eul: transforms dcm to Euler angles.
%
% INPUT
%   Cbn: must be b->n
%
% OUTPUT
%   euler: 3x1 Euler angles [roll pitch yaw] (rad, rad, rad).

Cb2n = Cb2n';

if nargin == 2
    
    if  strcmp(seq,'312')
        roll =  atan2(-Cb2n(1,3),Cb2n(3,3));
        pitch =   asin(Cb2n(2,3));
        yaw =  atan2(-Cb2n(2,1),Cb2n(2,2));
    elseif strcmp(seq,'321')
        roll =  atan2(Cb2n(2,3),Cb2n(3,3));
        pitch =  - asin(Cb2n(1,3));
        yaw =  atan2(Cb2n(1,2),Cb2n(1,1));
    else
        error("un-supported eular sequence");
    end
else
    % 321 sequence
    roll =  atan2(Cb2n(2,3),Cb2n(3,3));
    pitch =  - asin(Cb2n(1,3));
    yaw =  atan2(Cb2n(1,2),Cb2n(1,1));
end

eul(1) = roll;
eul(2) = pitch;
eul(3) = yaw;

