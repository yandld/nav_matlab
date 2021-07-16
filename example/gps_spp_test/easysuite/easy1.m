%EASY1	  Computation of the essential parameter: 
%              seconds of week, sow, also named tow

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/07/27  $
% revision: 2.0  2015/09/13

%RINEX version 3.03

% Note some important changes in RINEX version 3.03 compared to version 2.10: 
% Every line with epoch time starts with a >. This character is useful to maintain 
% synchronous reading. Next, the actual numbers of the satellites are removed 
% from this line and in stead included as the first column in the observation block

% The 0 after the number of seconds indicates that the file contains static
% observations and 8 the number of observed Svs which also is the number 
% of rows in the subsequent observation block

%A copy of line in the RINEX file  log_m24h.15o
%> 2015 09 03 07 45 26.0000000  0  8      -0.000125551060

% Compute sow for first epoch in the observation file
jd = julday(2015,9,3,7+45/60+26/3600);
[week,sow] = gps_time(jd)%;
%%%%%%%%%%%%%%%%%%%%% end easy1.m %%%%%%%%%%%%%%%



