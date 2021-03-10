function [p, v, q] = ch_nav_equ_local_tan(p, v, q ,acc, gyr, dt, gN)
%  惯导解算更新，当地直角坐标系，不考虑地球重力
% p         位置 XYZ 单位 m
% v          速度 XYZ 单位 m/s
% q         Qb2n姿态,四元数表示
% acc      比力， 加速度计测量值 单位  (m/s^2), 
% gyr      角速度 (rad/s)]
% dt        dt (s) 积分间隔如 0.01s
% gn       当地重力向量

old_v = v;

sf = acc;

%  姿态结算
q = ch_att_upt(q, gyr, dt);


% 速度解算
sf = ch_qmulv(q, sf);
sf = sf + gN;
v = old_v + dt *sf;

% 位置解算
p = p + (old_v + v) *dt/2;

end


% 
% 
% function x = ch_nav_equ_local_tan(x ,u, dt, gN)
% 
% persistent a_old;
% if isempty(a_old)
%    a_old= u(1:3);
% end
%    
% old_v = x(4:6);
% 
% a_new =u(1:3); 
% %sf = sf + 0.5*cross(u(4:6)*dt, sf);
% 
% %  姿态结算
% gyr = u(4:6);
% q_old = x(7:10);
% x(7:10) = ch_att_upt(x(7:10), gyr, dt);
% q_new = x(7:10);
% 
% % 速度解算
% 
% x(4:6) = old_v + ((ch_qmulv(q_new, a_new) + ch_qmulv(q_old, a_old) )/2 + gN) *dt;
% 
% % 位置解算
% x(1:3) = x(1:3) + (old_v + x(4:6)) *dt/2;
% a_old = a_new;
% end
% 
% 
