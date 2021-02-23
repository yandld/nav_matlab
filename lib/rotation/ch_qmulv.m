function vout = ch_qmulv(q, vin)
% 向量通过四元数做3D旋转
% 
% Inputs: q - Qb2n
%            vi - 需要旋转的向量
% Output: vout - output vector, such that vout = q*vin*conjugation(q)
% 
% See also  q2mat, qconj, qmul.
    qi = [0; vin];
    qo = ch_qmul(ch_qmul(q,qi),ch_qconj(q));
    vout = qo(2:4,1);
    