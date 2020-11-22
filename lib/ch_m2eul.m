function eul = ch_m2eul(Cbn)
%dcm2dul - Converts a coordinate transformation matrix to the
%corresponding set of Euler angles%
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 1/4/2012 by Paul Groves
%
% Inputs:
%   C       coordinate transformation matrix describing transformation from
%           beta(N) to alpha(B)
%
% Outputs:
%   eul     Euler angles describing rotation from beta to alpha in the 
%           order roll, pitch, yaw(rad)


% Begins

% Calculate Euler angles using (2.23)
eul(1,1) = atan2(Cbn(2,3),Cbn(3,3));  % roll
eul(2,1) = - asin(Cbn(1,3));        % pitch
eul(3,1) = atan2(Cbn(1,2),Cbn(1,1));  % yaw

% Ends