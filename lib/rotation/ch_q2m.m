function Cb2n = ch_q2m(Qb2n)
% 四元数转姿态阵
%
% Input: Qb2n
% Output: Cb2n
%
    q11 = Qb2n(1)*Qb2n(1); q12 = Qb2n(1)*Qb2n(2); q13 = Qb2n(1)*Qb2n(3); q14 = Qb2n(1)*Qb2n(4); 
    q22 = Qb2n(2)*Qb2n(2); q23 = Qb2n(2)*Qb2n(3); q24 = Qb2n(2)*Qb2n(4);     
    q33 = Qb2n(3)*Qb2n(3); q34 = Qb2n(3)*Qb2n(4);  
    q44 = Qb2n(4)*Qb2n(4);
    Cb2n = [ q11+q22-q33-q44,  2*(q23-q14),     2*(q24+q13);
            2*(q23+q14),      q11-q22+q33-q44, 2*(q34-q12);
            2*(q24-q13),      2*(q34+q12),     q11-q22-q33+q44 ];

        