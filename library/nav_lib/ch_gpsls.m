% GPS 伪距最小二乘法求解， 状态量为 X Y Z B(钟差)

function [pos, b, norm_dp, G] = ch_gpsls(pos, b,  sv_pos,  pr)

B1=1;
END_LOOP=100;
%卫星个数
n = size(sv_pos, 2);

if n < 4
    return
end
    b0 = b;
    while (END_LOOP > B1)
        % 获得当前位置与各个基站的距离
        r = vecnorm(sv_pos - pos);
        
        % 求得H矩阵
        H = (sv_pos - pos) ./ r;
        H =-H';
        
        H = [H(:,1:3),  ones(n,1)];
        
        dp = ((pr - r) + b0 - b)';
        
        % 迭代用户距离
        delta =  (H'*H)^(-1)*H'*dp;
        pos = pos + delta(1:3);
        b = b + delta(4);

         norm_dp = norm(dp);
        G = H;
        
        END_LOOP = vnorm(delta(1:3));
    end%End of While
end


