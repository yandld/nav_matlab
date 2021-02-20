function [Cb2n_312, Cb2n_321] = ch_eul2m(att)
% Convert Euler angles to direction cosine matrix(DCM).
%
% Input: att - att=[pitch; roll; yaw] in radians  att: 绕X,Y,Z旋转的角度，单位：rad
% 对于312顺序，对应[pitch roll yaw]
% 对于321顺序，对应[roll pitch yaw]
% Outputs: 
% Cb2n_312:  312顺序下的Cb2n
% Cb2n_321:  321顺序下的Cb2n

    s = sin(att); c = cos(att);
    si = s(1); sj = s(2); sk = s(3); 
    ci = c(1); cj = c(2); ck = c(3);
    Cb2n_312 = [ cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk;
            cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck;
           -ci*sj,           si,     ci*cj           ];
    if nargout==2  % dual Euler angle DCM
        Cb2n_321 = [ cj*ck, si*sj*ck-ci*sk, ci*sj*ck+si*sk;
                 cj*sk, si*sj*sk+ci*ck, ci*sj*sk-si*ck;
                -sj,    si*cj,          ci*cj            ];
    end
    