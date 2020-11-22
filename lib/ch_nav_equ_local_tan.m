
%%  function call: x=ch_nav_equ_local_tan(x,u,dt)
% x         Current navigation state [position (NED), velocity (NED), attitude (Quaternion, b2n)]
% u         Measured inertial quantities [Specific force (m/s^2), Angular velocity (rad/s)]
% dt        Sampling period (s)
% gn      Graivity

function x = ch_nav_equ_local_tan(x ,u, dt, gn)
old_v = x(4:6);
sf =u(1:3); 

%  姿态结算
gyr = u(4:6);
x(7:10) = ch_qintg(x(7:10), gyr, dt);

% 速度解算
q = x(7:10);
sf = ch_qmulv(q, sf);
sf = sf - gn;
x(4:6) = old_v + dt *sf;

% 位置解算
x(1:3) = x(1:3) + (old_v + x(4:6)) *dt/2;

end

