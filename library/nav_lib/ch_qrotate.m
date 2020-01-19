function vo = ch_qrotate(q, vi)

    qi = [0; vi];
    qo = ch_qmul(ch_qmul(ch_qconj(q), qi), q);
    vo = qo(2:4,1);