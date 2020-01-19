
function out = ch_qintg(in, gyr, dt)

%专业的做法
 rv = gyr*dt;
 dq = ch_rv2q(rv);

%专业的做法2
%     rv = gyr * dt;
%     n_rv = norm(rv);
%     dq(1) = cos(n_rv / 2);
%     dq(2) = sin(n_rv / 2) * rv(1) / n_rv;
%     dq(3) = sin(n_rv / 2) * rv(2) / n_rv;
%     dq(4) = sin(n_rv / 2) * rv(3) / n_rv;

%不专业的做法
%                  dq(1) = 1;
%                  dq(2) = rv(1)*0.5;
%                  dq(3) = rv(2)*0.5;
%                  dq(4) = rv(3)*0.5;

out = ch_qmul(in, dq);
out = ch_qnormlz(out);
end

