function [Xs,Ys,Zs,Deltat]=comsatpos(t,toc,a0,a1,a2,Crs,Delta_n,M0,Cuc,e,Cus,sqrtA,toe,Cic,OMEGA,Cis,i0,Crc,omega,OMEGA_DOT,iDOT)
%摄动改正后的升交距角uk、卫星矢径rk和轨道倾角ik
%卫星在轨道面坐标系中的坐标x,y
%发射时刻升交点的经度L
%卫星在地固坐标系下坐标Xs,Ys,Zs
%观测时间t，参考时刻toe，a0,a1,a2
%16参数
%Crs Delta_n M0  
%Cuc e Cus sqrtA 
%toe Cic  OMEGA Cis
%i0 Crc omega OMEGA_DOT  iDOT
%
%以下为计算代码
%1.计算卫星运行的平均角速度
n0=sqrt(3.986005E+14)/(sqrtA.^3);
n=n0+Delta_n;
%2.计算信号发射时卫星的平近点角
Deltat=a0+a1*(t-toc)+a2*(t-toc).^2;%t为未做钟差改正的观测时刻
t=t-Deltat;%更新t为做钟差改正后的值
tk=t-toe;%归化时间
% if tk>302400
%     tk=tk-604800;
% elseif tk<-302400
%     tk=tk+604800;
% else
%     tk=tk+0;
% end
Mk=M0+n*tk;
%3.计算偏近点角（迭代）
%E=M+e*sin(E);
ed(1)=Mk;
for i=1:4
   ed(i+1)=Mk+e*sin(ed(i));
end
Ek=ed(5);
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
L=OMEGA+(OMEGA_DOT-7.29211567E-5)*tk-7.292115E-5*toe;
%10.计算卫星在地固坐标系下坐标
Xs=x*cos(L)-y*cos(ik)*sin(L);
Ys=x*sin(L)+y*cos(ik)*cos(L);
Zs=y*sin(ik);
% DeltaX=Xs-(-20274509.129)
% DeltaY=Ys-(13349329.456)
% DeltaZ=Zs-(-10661361.857)
end