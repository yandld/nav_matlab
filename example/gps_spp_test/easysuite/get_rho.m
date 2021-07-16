function [tcorr,rho,X_ECEF] = get_rho(tR_RAW,pseudorange,Eph,X_receiver)
%GET_RHO  Calculation of distance in ECEF system between
%	      satellite and receiver at time tR_RAW given the
%	      ephemeris Eph.

%Kai Borre 04-01-96
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 1997/09/23  $

% Initial assigment of constants
vlight = 299792458;	         % vacuum speed of light in m/s
Omegae = 7.292115147e-5;	 % rotation rate of the earth in rad/s

%signal sended time=signal received time-signal transmission time
tx_RAW = tR_RAW-pseudorange/vlight;

%time correction acording to system time
toc = Eph(21);
      dt = check_t(tx_RAW-toc);
      tcorr = (Eph(2)*dt + Eph(20))*dt + Eph(19);
      tx_GPS = tx_RAW-tcorr;
      dt = check_t(tx_GPS-toc);
      tcorr = (Eph(2)*dt + Eph(20))*dt + Eph(19);
      tx_GPS = tx_RAW-tcorr; 

% satellite position 
X = satpos(tx_GPS, Eph);

%geometric range
rho = norm(X-X_receiver(1:3));

%satellite coordinates into inertial coordinate system
omegatau = Omegae*rho/vlight;
R3 = [ cos(omegatau) sin(omegatau) 0;
      -sin(omegatau) cos(omegatau) 0;
              0	       0	     1];
X_ECEF = R3*X;

%geometric range acording to inertial coordinate system
rho = norm(X_ECEF-X_receiver(1:3));
%%%%%%%%% end get_rho.m %%%%%%%%%
