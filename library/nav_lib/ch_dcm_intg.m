
function Cb2n_out = ch_dcm_intg(Cb2n_in, gyr, dt)
    theta = gyr*dt;
    n_theta = norm(theta);

    % 不专业的做法
    % dm = skew_symmetric(theta);
    %Cb2n_out = Cb2n_in + dm;

    % 方向余弦微分方程求解
    dm = eye(3) + sin(n_theta) / n_theta * skew_symmetric(theta)  + (1- cos(n_theta)) / n_theta^(2) * skew_symmetric(theta)^(2);
    Cb2n_out = Cb2n_in * dm;
end

