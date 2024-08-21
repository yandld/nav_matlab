function [att, attr] = m2att(Cnb)
% Convert direction cosine matrix(DCM) to Euler attitude angles.
%
% Prototype: [att, attr] = m2att(Cnb)
% Input: Cnb - DCM from navigation-frame(n) to body-frame(b)
% Outputs: att - att=[pitch; roll; yaw] in radians, in yaw->pitch->roll
%                (3-1-2) rotation sequence
%          attr - in yaw->roll->pitch (3-2-1) rotation sequence
% Test:
%   att0=randn(3,1)/10; [Cnb,Cnbr]=a2mat(att0); att=m2att(Cnb); [~,attr]=m2att(Cnbr); [att0, att, attr]
%
% See also  a2mat, a2qua, m2qua, q2att, q2mat, attsyn, m2rv, incline.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2008
    att = [ asin(Cnb(3,2));
            atan2(-Cnb(3,1),Cnb(3,3)); 
            atan2(-Cnb(1,2),Cnb(2,2)) ];
    if nargout==2  % dual Euler angles
        attr = [ atan2(Cnb(3,2),Cnb(3,3)); 
                 asin(-Cnb(3,1)); 
                 atan2(Cnb(2,1),Cnb(1,1)) ];
    end