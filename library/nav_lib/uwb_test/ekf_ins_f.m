% Dynamical model function for the random sine signal demo

% Copyright (C) 2007 Jouni Hartikainen
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function x_n = ekf_ins_f(x,param)
    g = [0;0;9.7803698];
    dT = param(1);
	accel_meas = param(2:4);
	gyro_meas = param(5:7);
	roll = x(7);pitch = x(8);head = x(9);
	I = eye(3,3);
	O = zeros(3,3);
	%%DCM Transfrmation: N->B
	Rz = [cos(head),-sin(head),0;
		  sin(head),cos(head),0;
		   0,0,1];
	Ry = [cos(pitch),0,sin(pitch);
		   0,1,0;
		  -sin(pitch),0,cos(pitch)];
	Rx = [1,0,0;
		  0,cos(roll),-sin(roll);
		  0,sin(roll),cos(roll)];	  
	Cbn = Rz*Ry*Rx;
	Lbn = [I *[1;0;0], Rx*[0;1;0] , Ry*Rx*[0;0;1]];
	Lnb = inv(Lbn);
    A = [I,I*dT,O;
		     O,I, O;
			 O,O,I];
    B = [dT^2/2*I,O;
             dT*I,O;
             O,dT*I];
    R = Cbn';  %%%b->n
    x_n = A*x(1:9) +B*[R*accel_meas-g;Lnb*gyro_meas];
	while x_n(9) >= 2*pi     x_n(9) = x_n(9) - 2*pi;end
	while x_n(8) >= 2*pi     x_n(8) = x_n(8) - 2*pi;end
	while x_n(7) >= 2*pi     x_n(7) = x_n(7) - 2*pi;end
	while x_n(9) <= -2*pi     x_n(9) = x_n(9) + 2*pi;end
	while x_n(8) <= -2*pi     x_n(8) = x_n(8) + 2*pi;end
	while x_n(7) <= -2*pi     x_n(7) = x_n(7) + 2*pi;end
% 	(R*accel_meas-g)'

    