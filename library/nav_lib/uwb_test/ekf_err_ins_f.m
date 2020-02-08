% Dynamical model function for the random sine signal demo

% Copyright (C) 2007 Jouni Hartikainen
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [x_n, E, D] = ekf_err_ins_f(x, param)
    dt = param(1);
	accel_meas = param(2:4);
	roll = param(5);pitch = param(6);head = param(7);
	I = eye(3,3);
	O = zeros(3,3);
	
	%%DCM Transfrmation:N-->B
	Rz = [cos(head),-sin(head),0;
		  sin(head),cos(head),0;
		   0,0,1];
	Ry = [cos(pitch),0,sin(pitch);
		   0,1,0;
		  -sin(pitch),0,cos(pitch)];
	Rx = [1,0,0;
		  0,cos(roll),-sin(roll);
		  0,sin(roll),cos(roll)];	  
	Cnb = Rz*Ry*Rx;
	R = Cnb';
	Lbn = [I *[1;0;0], Rx*[0;1;0] , Ry*Rx*[0;0;1]];
	Lnb = inv(Lbn);
	accel_l = R*accel_meas;
	omiga = [0,accel_l(3),-accel_l(2);
		         -accel_l(3),0,accel_l(1);
				 accel_l(2),-accel_l(1),0];
    A = [I,I*dt,O,O,O;
		     O,I,-omiga*dt,R*dt, O;
			 O,O,I,O, R*dt;
			 O,O,O,I, O;
			 O,O,O,O, I;
			 ];
    x_n = A*x;
	E = A;
	D = [
		     dt^2*R/2,O,O,O,O;
			 O,dt*R,O,O,O;
			 O,O,dt*Lnb,O,O;
			 O,O,O,I,O;
			 O,O,O,O,I;
		    ];