function sv_pos = ch_sv_pos_rotate(sv_pos, tau)
%% 计算经过地球自转改正后的卫星位置
% Earth's rotation rate
omega_e = 7.2921151467e-5; %(rad/sec)
theta = omega_e * tau;
sv_pos = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1]*sv_pos;
end
