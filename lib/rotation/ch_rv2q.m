function q = ch_rv2q(rv)  % 等效旋转矢量转换为变换四元数
    nm2 = rv'*rv;  % 旋转矢量的模方
    if nm2<1.0e-8  % 如果模方很小，则可用泰勒展开前几项求三角函数
        q0 = 1-nm2*(1/8-nm2/384); 
        s = 1/2-nm2*(1/48-nm2/3840);
    else
        nm = sqrt(nm2);
        q0 = cos(nm/2); 
        s = sin(nm/2)/nm;
    end
    q = [q0; s*rv];

