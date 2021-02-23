function [eul_312, eul_321] = ch_m2eul(Cb2n)
% 将姿态阵转为欧拉角
% 复用严龚敏老师的m2att
%
% Input: 姿态阵Cb2n: b系->n系的坐标变换矩阵
% Outputs: 
% eul_312:   312(Z->X->Y)旋转顺序下的欧拉角:  att = [pitch(绕X轴) roll(绕Y轴) yaw(绕Z轴)]
% eul_321:   321(Z->Y->X)旋转顺序下的欧拉角:  att = [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]
    eul_312 = [ asin(Cb2n(3,2));
            atan2(-Cb2n(3,1),Cb2n(3,3)); 
            atan2(-Cb2n(1,2),Cb2n(2,2)) ];
    if nargout==2  % dual Euler angles
        eul_321 = [ atan2(Cb2n(3,2),Cb2n(3,3)); 
                 asin(-Cb2n(3,1)); 
                 atan2(Cb2n(2,1),Cb2n(1,1)) ];
    end
