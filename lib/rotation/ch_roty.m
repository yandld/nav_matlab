function Cb2n = ch_roty(theta)
% 3D初等旋转， theta为旋转角度，rad
Cb2n = [cos(theta), 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

end