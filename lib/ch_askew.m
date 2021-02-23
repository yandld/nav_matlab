function m = ch_askew(v) 
% 生成反对称矩阵
%
% Input: v - 3x1 vector
% Output: m - v的反对称阵
%                    |  0   -v(3)  v(2) |
%             m = | v(3)  0    -v(1) |
%                    |-v(2)  v(1)  0    |
    m = [ 0,     -v(3),   v(2); 
          v(3),   0,     -v(1); 
         -v(2),   v(1),   0     ];
      