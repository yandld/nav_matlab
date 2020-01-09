
function out = ch_qintg(in, gyr, dt)
    theta = gyr * dt;
    n = norm(theta);

    dq(1) = cos(n / 2);
    dq(2) = sin(n / 2) * theta(1) / n;
    dq(3) = sin(n / 2) * theta(2) / n;
    dq(4) = sin(n / 2) * theta(3) / n;

    %                  dq(1) = 1;
    %                  dq(2) = theta(1)*0.5;
    %                  dq(3) = theta(2)*0.5;
    %                  dq(4) = theta(3)*0.5;

    out = ch_qmul(in, dq);
    out = ch_qnormlz(out);
end

