% FIXME: 王博和我的函数貌似不一样： quatrotate 的结果 和 ch_qmulv(或老严的qmulv) 是不一样的 

function [bQn, pos, vel, q] = ins(w_b, f_b,  bQn, pos, vel, gravity, dt)
%% 捷联更新
rotate_vector = w_b*dt;
rotate_vector_norm = norm(rotate_vector);
if(rotate_vector_norm <1e-10) % fix nan issue
    q = [1 0 0 0];
else
    q = [cos(rotate_vector_norm/2); rotate_vector/rotate_vector_norm*sin(rotate_vector_norm/2)]';
end

% 姿态更新
bQn = ch_qmul(bQn, q); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
bQn = ch_qnormlz(bQn); %单位化四元数

% 速度更新
f_n = ch_qmulv(bQn, f_b')';
dv = (f_n + [0; 0; -gravity]); %比力方程
vel = vel + dv*dt;

% 位置更新
pos = pos + vel*dt;

bQn = bQn';
