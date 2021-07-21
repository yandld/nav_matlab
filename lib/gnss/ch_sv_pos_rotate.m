function SP = ch_sv_pos_rotate(SP, tau)
%% 计算经过地球自转改正后的卫星位置
% Earth's rotation rate
% SP: 卫星位置
omega_e = 7.2921151467e-5; %(rad/sec)
theta = omega_e * tau;
SP = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1]*SP;
end
