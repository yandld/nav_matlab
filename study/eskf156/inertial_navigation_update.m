function [Qb2n, pos, vel, dQ] = inertial_navigation_update(w_b, f_b, Qb2n, pos, vel, gravity, dt)
%% 捷联惯性导航系统更新
% 输入:
%   w_b: 机体坐标系角速度 (3x1 向量)
%   f_b: 机体坐标系比力 (3x1 向量)
%   Qb2n: 机体到导航坐标系的当前姿态四元数 (4x1 向量)
%   pos: 当前位置 (3x1 向量)
%   vel: 当前速度 (3x1 向量)
%   gravity: 重力加速度标量值
%   dt: 时间步长
% 输出:
%   Qb2n: 更新后的机体到导航坐标系姿态四元数
%   pos: 更新后的位置
%   vel: 更新后的速度
%   dQ: 姿态增量四元数

% 计算旋转向量
rotation_vector = w_b * dt;
rotation_magnitude = norm(rotation_vector);

% 使用小角度近似来优化四元数计算
if rotation_magnitude < 1e-8
    dQ = [1; rotation_vector/2];
else
    half_angle = rotation_magnitude / 2;
    dQ = [cos(half_angle); rotation_vector/rotation_magnitude * sin(half_angle)];
end

% 姿态更新
Qb2n = ch_qmul(Qb2n, dQ');

% 可选：单位化四元数（如果需要的话）
% Qb2n = ch_qnormlz(Qb2n);

% 速度更新
f_n = ch_qmulv(Qb2n, f_b);
dv = f_n + [0; 0; -gravity];
vel = vel + dv * dt;

% 位置更新
pos = pos + vel * dt;

end
