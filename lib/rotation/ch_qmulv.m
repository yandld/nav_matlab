function vo = ch_qmulv(q, vi)
% 向量通过四元数做3D旋转
% 
% Inputs: q - Qb2n
%            vi - 需要旋转的向量
% Output: vout - output vector, such that vout = q*vin*conjugation(q)
% 
% See also  q2mat, qconj, qmul.



%     qi = [0; vi];
%     qo = ch_qmul(ch_qmul(q,qi),ch_qconj(q));
%     vo = qo(2:4,1);

    qo1 =              - q(2) * vi(1) - q(3) * vi(2) - q(4) * vi(3);
    qo2 = q(1) * vi(1)                + q(3) * vi(3) - q(4) * vi(2);
    qo3 = q(1) * vi(2)                + q(4) * vi(1) - q(2) * vi(3);
    qo4 = q(1) * vi(3)                + q(2) * vi(2) - q(3) * vi(1);
    vo = vi;
    vo(1) = -qo1 * q(2) + qo2 * q(1) - qo3 * q(4) + qo4 * q(3);
    vo(2) = -qo1 * q(3) + qo3 * q(1) - qo4 * q(2) + qo2 * q(4);
    vo(3) = -qo1 * q(4) + qo4 * q(1) - qo2 * q(3) + qo3 * q(2);
    