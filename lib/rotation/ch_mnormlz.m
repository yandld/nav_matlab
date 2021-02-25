function Cb2n = ch_mnormlz(Cb2n)
% 姿态阵单位化，防止矩阵蠕变
    for k=1:5
%         Cnb = 0.5 * (Cnb + (Cnb')^-1);  % Algorithm 1 
        Cb2n = 1.5*Cb2n - 0.5*(Cb2n*Cb2n')*Cb2n;  % Algorithm 2 
    end
    
%   Algorithm 3:  [s,v,d] = svd(Cnb); Cnb = s*d';  % in = s*v*d' = s*d' * d*v*d';  out = s*d'