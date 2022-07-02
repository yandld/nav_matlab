function [X, dx, G] = ch_gpsls(X,  sat_pos,  pr)
% GPS 伪距最小二乘法求解， 状态量为 X Y Z B(钟差)
% X: X 值(1:3) 位置delta, (4) 用户钟差偏差
% G: 设计矩阵
% pr: 校正后的伪距
% sat_pos: 卫星位置矩阵
% delta: delta 值(1:3) 位置delta, (4) 用户钟差偏差

B1=1;
END_LOOP=100;
%卫星个数
n = size(sat_pos, 2);

if n < 4
    dx = 0;
    G = 0;
    return
end

    for loop = 1:10
        % 获得当前位置与各个基站的距离
        r = vecnorm(sat_pos - X(1:3));
        
        % 求得H矩阵
        H = (sat_pos - X(1:3)) ./ r;
        H =-H';
        
        H = [H(:,1:3),  ones(n,1)];
        
        dp = ((pr - r) -  X(4))';
        
        % 迭代用户距离
        dx =  (H'*H)^(-1)*H'*dp;
        X = X + dx;
        G = H;
        
    %    END_LOOP = vnorm(delta(1:3));
    end
    
    
end


