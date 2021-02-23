function q = ch_qnormlz(q)
% 四元数归一化
    q = q/norm(q);
    if(q(1)<0)
        q(1) = -q(1);
        q(2) = -q(2);
        q(3) = -q(3);
        q(4) = -q(4);
    end