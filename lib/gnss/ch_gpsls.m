function [X, delta, G] = ch_gpsls(X,  SVP,  rho)
% GPS 伪距最小二乘法求解， 状态量为 X Y Z B(钟差)
% X: X 值(1:3) 位置delta, (4) 用户钟差偏差
% rho: 校正后的伪距
% SVP: 卫星位置矩阵
% delta: delta 值(1:3) 位置delta, (4) 用户钟差偏差

B1=1;
END_LOOP=100;
%卫星个数
n = size(SVP, 2);

if n < 4
    delta = 0;
    G = 0;
    return
end
   X0 = X;

    for loop = 1:10
        % 获得当前位置与各个基站的距离
        r = vecnorm(SVP - X(1:3));
        
        % 求得H矩阵
        H = (SVP - X(1:3)) ./ r;
        H =-H';
        
        H = [H(:,1:3),  ones(n,1)];
        
        dp = ((rho - r) -  X(4))';
        
        % 迭代用户距离
        delta =  (H'*H)^(-1)*H'*dp;
        X = X + delta;
        G = H;
        
    %    END_LOOP = vnorm(delta(1:3));
    end
    
    delta = X - X0;
    
end


