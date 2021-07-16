function sv_dt=sv_clock_bias(t, toc, a0, a1, a2)
% 输入:
% a0 a1 a2 toc: 卫星时钟校正模型方程中3个参数，  toc: 第一数据块参考时间, 被用作时钟校正模型中时间参考点

% sv_dt： 卫星时钟偏差

sv_dt = a0+a1*(t-toc)+a2*(t-toc).^2;%t为未做钟差改正的观测时刻

end