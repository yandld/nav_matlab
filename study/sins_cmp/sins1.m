function q = sins1(q, w_b, dt)
    % 捷联更新
    rotate_vector = w_b*dt;
    rotate_vector_norm = norm(rotate_vector);
    if(rotate_vector_norm <1e-10) % fix nan issue
        dq = [1 0 0 0];
    else
        dq = [cos(rotate_vector_norm/2); rotate_vector/rotate_vector_norm*sin(rotate_vector_norm/2)]';
    end

    % 姿态更新
    q = ch_qmul(q, dq); %四元数更新（秦永元《惯性导航（第二版）》P260公式9.3.3）
    q = ch_qnormlz(q); %单位化四元数
end

