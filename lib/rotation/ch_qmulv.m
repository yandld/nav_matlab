function out = ch_qmulv(q, in)
    qi = [0;in];
    qo = ch_qmul(ch_qmul(q,qi),ch_qconj(q));
    out = qo(2:4,1);
    