function [acc_n, raw_acc_n] = body2nav(Qb2n, acc_b, GRAVITY)
% BODY2NAV 将体坐标系加速度转换到导航坐标系
%
% 输入:
%   Qb2n - 从体坐标系到导航坐标系的四元数 [qw, qx, qy, qz] (nx4)
%   acc_b - 体坐标系加速度 [ax, ay, az] (nx3)
%   GRAVITY - 重力加速度常数 (标量)
%
% 输出:
%   acc_n - 导航坐标系垂直方向加速度 (nx1)
%   raw_acc_n - 导航坐标系三轴加速度 (nx3)
%
% 示例:
%   [acc_n, raw_acc_n] = body2nav(Qb2n, acc_b, 9.81);

    % 获取数据行数
    n = size(acc_b, 1);
    
    % 初始化输出
    raw_acc_n = zeros(n, 3);
    
    % 计算导航系加速度
    for i = 1:n
        raw_acc_n(i,:) = qmulv(Qb2n(i,:), acc_b(i,:));
        raw_acc_n(i,3) = raw_acc_n(i,3) - GRAVITY; % 去除重力
    end
    
    % 只取垂直方向加速度
    acc_n = raw_acc_n(:,3);
end

function r = qmulv(q, v)
    % qmulv - 使用单位四元数旋转三维向量
    %
    % 输入:
    %   q - 单位四元数 [w, x, y, z] (1x4 或 4x1)
    %   v - 三维向量 [x, y, z] (1x3 或 3x1)
    %
    % 输出:
    %   r - 旋转后的三维向量 [x, y, z] (1x3)
    %
    % 注意:
    %   假设四元数 q 已归一化。为性能考虑，不进行归一化检查。

    % 确保输入是行向量
    if size(q, 1) > 1
        q = q'; % 转置为行向量
    end
    if size(v, 1) > 1
        v = v'; % 转置为行向量
    end

    % 提取四元数分量
    q0 = q(1); % 标量部分
    q1 = q(2); % x 分量
    q2 = q(3); % y 分量
    q3 = q(4); % z 分量

    % 提取向量分量
    vx = v(1);
    vy = v(2);
    vz = v(3);

    % 预计算常用项 (减少重复计算)
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q3 = q2 * q3;

    % 计算旋转后的向量
    r(1) = (q0q0 + q1q1 - q2q2 - q3q3) * vx + ...
           2 * (q1q2 - q0q3) * vy + ...
           2 * (q1q3 + q0q2) * vz;

    r(2) = 2 * (q1q2 + q0q3) * vx + ...
           (q0q0 - q1q1 + q2q2 - q3q3) * vy + ...
           2 * (q2q3 - q0q1) * vz;

    r(3) = 2 * (q1q3 - q0q2) * vx + ...
           2 * (q2q3 + q0q1) * vy + ...
           (q0q0 - q1q1 - q2q2 + q3q3) * vz;

    % 确保输出为行向量
    r = r(:)';
end
