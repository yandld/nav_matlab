function [XYZ, sv_dt]=ch_sat_pos(t, toc, a0, a1, a2, Crs, Delta_n, M0, Cuc, e, Cus, sqrtA, toe, Cic, OMEGA, Cis, i0, Crc, omega, OMEGA_DOT, iDOT)
% 输入:
% toe: 星历参考时间： 一套星历的有效期为toe前后4小时
% a0 a1 a2 toc: 卫星时钟校正模型方程中3个参数，  toc: 第一数据块参考时间, 被用作时钟校正模型中时间参考点


% 卫星星历参数(16个):
% sqrtA: 卫星轨道长半轴的平方根
% es: 轨道偏心率
% i0: toe时刻轨道倾角
% OMEGA0: 周内时等于0时刻的轨道升交点赤经
% omega: 轨道近地角距
% M0: toe时刻的平近点角
% Delta_n： 平均运动角速度校正值
% iDOT： 轨道倾角对时间的变化率
% OMEGA_DOT： 轨道升交点赤经对时间的变化率
% Cuc: 升交点角距余弦和校正振幅
% Cus: 升交点角距正弦和校正振幅
% Cic: 轨道倾角余弦和校正振幅
% Cis: 轨道倾角正弦和校正振幅

%输出:
% X, Y, Z ECEF下卫星位置
% sv_dt： 卫星时钟偏差

%变量: 
%摄动改正后的升交距角uk、卫星矢径rk和轨道倾角ik
%卫星在轨道面坐标系中的坐标x,y
%发射时刻升交点的经度L


%以下为计算代码
%1.计算卫星运行的平均角速度
n0=sqrt(3.986005E+14)/(sqrtA.^3);
n=n0+Delta_n;
%2.计算信号发射时卫星的平近点角
sv_dt = a0+a1*(t-toc)+a2*(t-toc).^2;%t为未做钟差改正的观测时刻
t = t - 0;%更新t为做钟差改正后的值
tk = t-toe;%归化时间
if tk>302400
    tk=tk-604800;
elseif tk<-302400
    tk=tk+604800;
else
    tk=tk+0;
end
Mk=M0+n*tk;
%3.计算偏近点角（迭代）
%E=M+e*sin(E);
ed(1)=Mk;
for i=1:3
   ed(i+1)=Mk+e*sin(ed(i));
end
Ek=ed(end);
%4.计算真近点角
Vk=atan2(sqrt(1-e.^2)*sin(Ek),(cos(Ek)-e));
%5.计算升交距角（未经改进时）
u=omega+Vk;
%6.计算摄动改正项
deltau=Cuc*cos(2*u)+Cus*sin(2*u);
deltar=Crc*cos(2*u)+Crs*sin(2*u);
deltai=Cic*cos(2*u)+Cis*sin(2*u);
%7.计算摄动改正后的升交距角uk、卫星矢径rk和轨道倾角ik
uk=u+deltau;
rk=(sqrtA.^2)*(1-e*cos(Ek))+deltar;
ik=i0+deltai+iDOT*tk;
%8.计算卫星在轨道面坐标系中的坐标
x=rk*cos(uk);
y=rk*sin(uk);
%9.计算发射时刻升交点的经度67
L=OMEGA+(OMEGA_DOT-7.2921151467e-5)*tk-7.2921151467e-5*toe;
%10.计算卫星在地固坐标系下坐标
XYZ(1)=x*cos(L)-y*cos(ik)*sin(L);
XYZ(2)=x*sin(L)+y*cos(ik)*cos(L);
XYZ(3)=y*sin(ik);
XYZ = XYZ';

end