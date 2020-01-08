function q = ch_qnormlz(q)
% Quaternion normalization, so ||qnb||=1.
%
% Prototype: qnb = qnormlz(qnb)
% Input: qnb - input quaternion whose norm may not be 1
% Output: qnb - input quaternion whose norm equals 1
%
% See also  vnormlz, mnormlz.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/09/2012
    q = q/norm(q);