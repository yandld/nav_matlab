%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% function call: x=ch_nav_equ_local_tan(x,u,dt)
% x         Current navigation state [position (NED), velocity (NED), attitude (Quaternion)]
% u         Measured inertial quantities [Specific force (m/s^2), Angular velocity (rad/s)]
% dt        Sampling period (s)
% g_t      Graivity(-9.81x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = ch_nav_equ_local_tan(x ,u, dt, g_t)

%%          Position and velocity        %%
p = x(1:3);
old_v = x(4:6);
sf =u(1:3); % specfic force


q = x(7:10);
sf = ch_q2m(q)' * sf;

% Subtract gravity, to obtain accelerations in tangent plane coordinates
sf = sf - g_t;

% Position and velocity update
v = eye(3)*dt *sf;
x(4:6) = old_v + v;
x(1:3) = x(1:3) + old_v*dt + v*dt/2;

%%        Attitude Quaternion 

omega=u(4:6);
x(7:10) = ch_qintg(x(7:10), omega, dt);

end

