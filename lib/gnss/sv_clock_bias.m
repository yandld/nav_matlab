function dtr = sv_clock_bias(t, toc, a0, a1, a2, e, sqrtA, toe, Delta_n, M0)
% 输入:
% a0 a1 a2 toc: 卫星时钟校正模型方程中3个参数，  toc: 第一数据块参考时间, 被用作时钟校正模型中时间参考点

% dtr： 卫星时钟偏差

dtr = a0+a1*(t-toc)+a2*(t-toc).^2;%t为未做钟差改正的观测时刻


F = -4.442807633e-10;
mu = 3.986005e14;
A = sqrtA^2;
cmm = sqrt(mu/A^3); % computed mean motion
tk = t - toe;
% account for beginning or end of week crossover
if (tk > 302400)
    tk = tk-604800;
end
if (tk < -302400)
    tk = tk+604800;
end
% apply mean motion correction
n = cmm + Delta_n;

% Mean anomaly
mk = M0 + n*tk;

% solve for eccentric anomaly
Ek = mk;
Ek = mk + e*sin(Ek);
Ek = mk + e*sin(Ek);
Ek = mk + e*sin(Ek);

% dsv 时间为s
dtr = dtr + F*e*sqrtA*sin(Ek);
 
end