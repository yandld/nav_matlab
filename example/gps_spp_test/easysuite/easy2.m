%EASY2	 Convert observation time into sow.
%             We read the corresponding RINEX navigation file 
%             and reformat the data into the Matlab matrix Eph.
%            For given SV we find the corresponding column in Eph 
%            and call the basic satpos function

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/07/27  $
% Revision 2.0, Februeary 13, 2016

%RINEX version 3.03

%A copy of line in the RINEX file  log_m24h.15o
%> 2015 09 03 07 45 26.0000000  0  8      -0.000125551060

% Compute sow for first epoch in observation file
jd = julday(2015,9,3,7+45/60+26/3600);
[week,sow] = gps_time(jd)%;

% Read RINEX ephemerides file, version 3.30,  and convert to
% internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');
          
svs = [5 6 31 24 25 12 29]; %  there is no eph for sv 2
m = length(svs);
col_Eph = zeros(m);
sat = zeros(3,m);
for t = 1:m
    col_Eph(t) = find_eph(Eph,svs(t),sow);
    sat(1:3,t) = satpos(sow,Eph(:,col_Eph(t)));
end

sat     % positions of svs in ECEF system
%%%%%%%%%%%%%%%%%%%%% end easy2.m %%%%%%%%%%%%%%%



