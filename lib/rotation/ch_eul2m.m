function [Cb2n_312, Cb2n_321] = ch_eul2m(att)
% 将欧拉角转换为姿态阵
% 复用严龚敏老师的a2mat
%
% Input: att  单位：rad
% 对于312((Z->X->Y))顺序，对应att = [pitch(绕X轴) roll(绕Y轴)  yaw(绕Z轴)]
% 对于3211(Z->Y->X)顺序，对应att = [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]
% Outputs: 
% Cb2n_312:  312欧拉角顺序下转换后的Cb2n
% Cb2n_321:  321欧拉角顺序下转换后的Cb2n

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
    