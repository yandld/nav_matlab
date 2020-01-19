function m = ch_rv2m(rv)   % 等效旋转矢量转换为变换矩阵
	nm2 = rv'*rv;  % 旋转矢量的模方
    if nm2<1.e-8   % 如果模方很小，则可用泰勒展开前几项求三角函数
        a = 1-nm2*(1/6-nm2/120); b = 0.5-nm2*(1/24-nm2/720);  % a->1, b->0.5
    else
        nm = sqrt(nm2);
        a = sin(nm)/nm;  b = (1-cos(nm))/nm2;
    end
    VX = ch_askew(rv);
    m = eye(3) + a*VX + b*VX^2;