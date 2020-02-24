%%
% anchor_pos: anchor pos:  3xN (N是基站数量)
% p 用户位置
% pr 伪距

function p = ch_triangulate(anchor_pos, p,  pr)

% 基站个数
n = size(anchor_pos, 2);

% 当前估计的到各个基站的距离
r = vecnorm(anchor_pos - p);

% 计算几何矩阵
H = (anchor_pos - p) ./ r;
H =-H';

% 更新位置
p =  p + (H'*H)^(-1)*H'*(pr - r)';
end


