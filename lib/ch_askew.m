function m = ch_askew(v)  % 三维实向量的反对称阵
    m = [ 0,     -v(3),   v(2); 
          v(3),   0,     -v(1); 
         -v(2),   v(1),   0     ];
      