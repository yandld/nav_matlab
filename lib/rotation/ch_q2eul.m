function [eul_312, eul_321] =  ch_q2eul(Qb2n)
% 四元数转欧拉角
%
% Input: 四元数Qb2n
% Outputs:
% eul_312:   312(Z->X->Y)旋转顺序下的欧拉角:  att = [pitch(绕X轴) roll(绕Y轴) yaw(绕Z轴)]
% eul_321:   321(Z->Y->X)旋转顺序下的欧拉角:  att = [roll(绕X轴) pitch(绕Y轴)  yaw(绕Z轴)]

q0 = Qb2n(1);
q1 = Qb2n(2);
q2 = Qb2n(3);
q3 = Qb2n(4);


roll =  -atan2( 2*( q1*q3 - q0*q2 ) , q0*q0 - q1*q1 - q2*q2 + q3*q3);
pitch = asin( 2*(q0*q1 + q2*q3) );
yaw = -atan2(2*( q1*q2 - q0*q3 ), q0*q0 - q1*q1 + q2*q2 - q3*q3);
eul_312 = [pitch; roll; yaw];

if nargout==2  % dual Euler angles
    roll =  atan2( 2*( q0*q1 + q2*q3 ) , 1 - 2*q1*q1 - 2*q2*q2);
    pitch = asin( 2*(q0*q2 - q1*q3) );
    yaw = atan2(2*( q0*q3 + q1*q2 ), 1 - 2*q2*q2 - 2*q3*q3);
    eul_321 = [roll; pitch; yaw];
end


