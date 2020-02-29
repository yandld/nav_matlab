% 迭代式多边测距

function p = ch_triangulate(anchor_pos, p,  pr)

% 基站个数
n = size(anchor_pos, 2);

% 获得当前位置与各个基站的距离
r = vecnorm(anchor_pos - p);

% 求得H矩阵
H = (anchor_pos - p) ./ r;
H =-H';

% 迭代用户距离
p =  p + (H'*H)^(-1)*H'*(pr - r)';


end


