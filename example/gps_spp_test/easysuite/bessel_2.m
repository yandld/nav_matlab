function [s12, A1, A2] = bessel_2(phi1d, phi1m, phi1s, l1d, l1m, ...
                        l1s, phi2d, phi2m, phi2s, l2d, l2m, l2s, a, finv)
%BESSEL_2 Solution of the inverse geodetic problem according to
%         the Bessel-Helmert method as described in Zhan Xue-Lian.
%         Given two points with coordinates (phi1, l1) and 
%         (phi2,l2). Per definition we always have l2 > l1.
%         The given reference ellipsoid has semi-major
%         axis a and inverse flattening finv. The coordinates
%         are in the format degree, minute, and second with decimals. 
%
%         Zhan Xue-Lian (1985) The nested coefficient method for
%            accurate solutions of direct and inverse geodetic 
%            problems with any length. Proceedings of the 7th
%            International Symposium on Geodetic Computations.
%            Cracow, June 18--21, 1985. Institute of Geodesy and
%            Cartography. Wasaw, Poland, ul. Jasna 2/4
%
%         For a good background of the problems, see 
%
%  	      Bodem\"uller, H.(1954): Die geod\"atischen Linien des 
%	         Rotationsellipsoides und die L\"osung der geod\"atischen
%	         Hauptaufgaben f\"ur gro\ss{}e Strecken unter 
%            besonderer Ber\"ucksichtigung der Bessel-Helmertschen
%            L\"osungsmethode.
%	         Deutsche Geod\"atische Kommission, Reihe B, Nr. 13.

%Kai Borre, January 25, 1999
%Copyright (c) by Kai Borre
%$Revision 1.0 $  $Date:1999/01/25  $

if nargin == 0
   phi1  = dms2rad(50,0,0);
   l1 = dms2rad(10,0,0);
   phi2 = dms2rad(-62,57,3.203824);
   l2 = dms2rad(105,5,38.299430);
   a = 6378388;
   finv = 297;
else
   phi1 = dms2rad(phi1d,phi1m,phi1s);
   l1 = dms2rad(l1d,l1m,l1s);
   phi2 = dms2rad(phi2d,phi2m,phi2s);
   l2 = dms2rad(l2d,l2m,l2s);
end
f = 1/finv; 
ex2 = (2-f)*f/(1-f)^2;

%Reduced latitudes
u1 = atan((1-f)*tan(phi1)); 
u2 = atan((1-f)*tan(phi2));
deltaomega_old = 0;
deltaomega_new = 1;

while abs(deltaomega_old-deltaomega_new) > 1.e-12
   deltaomega_old = deltaomega_new;
   omega = l2-l1+deltaomega_old;
   sigma = atan2(sqrt((cos(u2)*sin(omega))^2+...
      (cos(u1)*sin(u2)-sin(u1)*cos(u2)*cos(omega))^2),...
      sin(u1)*sin(u2)+cos(u1)*cos(u2)*cos(omega)); 
   cosun = cos(u1)*cos(u2)*sin(omega)/sin(sigma);   
   sinun2 = 1-cosun^2;   
   if sinun2 == 0
      sigmam = acos(cos(sigma)-2);
   else
      sigmam = acos(cos(sigma)-2*sin(u1)*sin(u2)/sinun2);   
   end
   v = f*sinun2/4;
   K3 = v*(1+f+f^2-v*(3+7*f-13*v));
   deltaomega_new = (1-K3)*f*cosun*(sigma+K3*sin(sigma)*(cos(sigmam)+...
      K3*cos(sigma)*cos(2*sigmam)));
end

t = ex2*sinun2/4;
K1 = 1+t*(1-t*(3-t*(5-11*t))/4);
K2 = t*(1-t*(2-t*(37-94*t)/8));
deltasigma = K2*sin(sigma)*(cos(sigmam)+...
   K2*(cos(sigma)*cos(2*sigmam)+...
   K2*(1+2*cos(2*sigma))*cos(3*sigmam)/6)/4);
s12 = K1*(1-f)*a*(sigma-deltasigma);
A1 = atan2(cos(u2)*sin(omega),...
   cos(u1)*sin(u2)-sin(u1)*cos(u2)*cos(omega));
A2 = atan2(cos(u1)*sin(omega),...
   cos(u1)*sin(u2)*cos(omega)-sin(u1)*cos(u2));
       

%----------------------------------------------

function result = dms2rad(deg,min,sec)
% Conversion of degrees, minutes, and seconds to radians

neg_arg = 'FALSE';
if deg < 0
   neg_arg = 'TRUE ';
   deg = -deg;
end
arg = deg+min/60+sec/3600;
result = arg*pi/180;
if strcmp(neg_arg, 'TRUE ')
   result = -result;
end

%%%%%%%%%%%%%%%%% end bessel_2.m  %%%%%%%%%%%%%%%%%%%%%%%%%%%
