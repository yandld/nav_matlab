function vo = ch_qmulv(q, vi)
    qi = [0;vi];
    qo = qmul(qmul(q,qi),qconj(q));
    vo = qo(2:4,1);
    