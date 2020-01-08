
%% 生成 叉积反对称矩阵

function m=skew_symmetric(x)
m=[
    0    -x(3)    x(2) ;
    x(3)     0    -x(1) ;
    -x(2)  x(1)    0 ];
end