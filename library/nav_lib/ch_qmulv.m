function vo = ch_qmulv(q, vi)
    qi = [0;vi];
    qo = ch_qmul(ch_qmul(q,qi),ch_qconj(q));
    vo = qo(2:4,1);
    