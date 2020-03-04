% GPS 伪距最小二乘法求解， 状态量为 X Y Z B(钟差)

function x = ch_gpsls(x, sv_pos,  pr)

B1=1;
END_LOOP=100;
%卫星个数
n = size(sv_pos, 2);

if n < 4
    return
end
    
    while (END_LOOP > B1)
        % 获得当前位置与各个基站的距离
        r = vecnorm(sv_pos - x(1:3));
        
        % 求得H矩阵
        H = (sv_pos - x(1:3)) ./ r;
        H =-H';
        
        H = [H(:,1:3),  ones(n,1)];
        
        b = ((pr - r) - x(4))';
        
        % 迭代用户距离
        delta =  (H'*H)^(-1)*H'*b;
        x = x + delta;
        
        END_LOOP = vnorm(delta(1:3));
    end%End of While
end


