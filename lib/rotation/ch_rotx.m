function Cb2n = ch_rotx(theta)
% 3D初等旋转， theta为旋转角度，rad
Cb2n = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];

end