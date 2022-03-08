%
function [nQb, pos, vel, q] = ins(w_b, f_b,  nQb, pos, vel, gravity, dt)
rotate_vector = w_b*dt;
rotate_vector_norm = norm(rotate_vector);
if(rotate_vector_norm <1e-10) % fix nan issue
    q = [1 0 0 0];
else
    q = [cos(rotate_vector_norm/2); rotate_vector/rotate_vector_norm*sin(rotate_vector_norm/2)]';
end

% 姿态更新
nQb = quatmultiply(nQb, q); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
nQb = quatnormalize(nQb); %单位化四元数
bQn = quatinv(nQb); %更新bQn

% 速度更新
f_n = quatrotate(bQn, f_b')';
dv = (f_n + [0; 0; -gravity]); %比力方程
vel = vel + dv*dt;

% 位置更新
pos = pos + vel*dt;

end

