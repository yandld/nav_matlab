
function Cb2n_out = ch_dcm_intg(Cb2n_in, gyr, dt)
% 专业的做法1
rv = gyr*dt;
dm = ch_rv2m(rv);

%专业的做法，更新旋转矩阵
Cb2n_out = Cb2n_in * dm;

% 不专业的做法
%  rv = gyr*dt;
% dm = skew_symmetric(rv);
% Cb2n_out = Cb2n_in + dm;

end

