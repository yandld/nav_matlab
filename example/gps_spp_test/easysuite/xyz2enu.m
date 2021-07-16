function [e,n,u] = xyz2enu(phi,lambda,x,y,z)
%XYZ2ENU  Transformation of [x;y;z] vector from geocentric to local
%   	  system. The local system has origin at (phi, lambda) 
%         that are given in degrees

%Kai Borre 21-07-99
%Copyright (c) by Kai Borre
%$Revision: 2.0 $  $Date: 2001/10/28  $

phi = phi*pi/180;
lambda = lambda*pi/180;
cl = cos(lambda);  sl = sin(lambda);
cb = cos(phi);	    sb = sin(phi);
F = [-sl -sb*cl cb*cl;
      cl -sb*sl cb*sl;
      0	  cb      sb];
local_vect = F'*[x; y; z];
e = local_vect(1);
n = local_vect(2);
u = local_vect(3);
%%%%%%%%% end xyz2enu.m %%%%%%%%%
