% 最小二乘法多边测距

function p = ch_multilateration(anchor_pos,  pr)

pr = pr(1:size(anchor_pos, 2));

b = vecnorm(anchor_pos).^(2) - pr.^(2);
b = b(1:end-1) - b(end);
b = b';

A =  anchor_pos - anchor_pos(:,end);
A = A(:,1:end-1)'*2;

p = (A'*A)^(-1)*A'*b;


end


