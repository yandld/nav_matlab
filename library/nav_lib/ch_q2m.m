function Cbn = ch_q2m(q)
%q2dcm - Converts a quatnrion to the
%corresponding Cbn
%
%
% Inputs:
%   q       coordinate transformation matrix describing transformation from
%           beta(N) to alpha(B)
%
% Outputs: Cbn, coordinate transformation matrix describing transformation from beta(N) to alpha(B)
%   

    q11 = q(1)*q(1); q12 = q(1)*q(2); q13 = q(1)*q(3); q14 = q(1)*q(4); 
    q22 = q(2)*q(2); q23 = q(2)*q(3); q24 = q(2)*q(4);     
    q33 = q(3)*q(3); q34 = q(3)*q(4);  
    q44 = q(4)*q(4);
    Cbn = [ q11+q22-q33-q44,  2*(q23+q14),             2*(q24-q13);
            2*(q23-q14),                q11-q22+q33-q44,    2*(q34+q12);
            2*(q24+q13),               2*(q34-q12),             q11-q22-q33+q44 ];
        