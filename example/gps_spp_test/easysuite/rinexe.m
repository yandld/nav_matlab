function rinexe(ephemerisfile, outputfile)
%RINEXE Reads a RINEX Navigation Message file version 3.03 and
%	        reformats the data into a matrix with 21 rows and a column 
%           for each satellite.  The matrix is stored in outputfile

%Typical call: rinexe('pta.96n','pta.nav')

%Kai Borre 04-18-96
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 1997/09/24  $
% Revision September 11, 2015 at Samara

% Units are either seconds, meters, or radians
fide = fopen(ephemerisfile);
head_lines = 0;
while 1  % We skip header
   head_lines = head_lines+1;
   line = fgetl(fide);
   answer = findstr(line,'END OF HEADER');
   if ~isempty(answer), break;	end;
end;
head_lines;
noeph = -1;
while 1
   noeph = noeph+1;
   line = fgetl(fide);
   if line == -1, break;  end
end;
noeph = noeph/8
frewind(fide);
for i = 1:head_lines, line = fgetl(fide); end;

% Set aside memory for the input
svprn	 = zeros(1,noeph);
weekno	 = zeros(1,noeph);
t0c	 = zeros(1,noeph);
tgd	 = zeros(1,noeph);
aodc	 = zeros(1,noeph);
toe	 = zeros(1,noeph);
af2	 = zeros(1,noeph);
af1	 = zeros(1,noeph);
af0	 = zeros(1,noeph);
aode	 = zeros(1,noeph);
deltan	 = zeros(1,noeph);
M0	 = zeros(1,noeph);
ecc	 = zeros(1,noeph);
roota	 = zeros(1,noeph);
toe	 = zeros(1,noeph);
cic	 = zeros(1,noeph);
crc	 = zeros(1,noeph);
cis	 = zeros(1,noeph);
crs	 = zeros(1,noeph);
cuc	 = zeros(1,noeph);
cus	 = zeros(1,noeph);
Omega0	 = zeros(1,noeph);
omega	 = zeros(1,noeph);
i0	 = zeros(1,noeph);
Omegadot = zeros(1,noeph);
idot	 = zeros(1,noeph);
accuracy = zeros(1,noeph);
health	 = zeros(1,noeph);
fit	 = zeros(1,noeph);

for i = 1:noeph
   line = fgetl(fide);	  %
   svprn(i) = str2num(line(2:3)); % a G for GPS in front is omitted
   year = line(5:8);
   month = line(10:11);
   day = line(13:14);
   hour = line(16:17);
   minute = line(19:20);
   second = line(22:23);
   af0(i) = str2num(line(24:42));
   af1(i) = str2num(line(43:61));
   af2(i) = str2num(line(62:80));
   line = fgetl(fide);	  %
   IODE = line(5:23);
   crs(i) = str2num(line(24:42));
   deltan(i) = str2num(line(43:61));
   M0(i) = str2num(line(62:80));
   line = fgetl(fide) ;	  %
   cuc(i) = str2num(line(5:23));
   ecc(i) = str2num(line(24:42));
   cus(i) = str2num(line(43:61));
   roota(i) = str2num(line(62:80));
   line=fgetl(fide);
   toe(i) = str2num(line(5:23));
   cic(i) = str2num(line(24:42));
   Omega0(i) = str2num(line(43:61));
   cis(i) = str2num(line(62:80));
   line = fgetl(fide);	    %
   i0(i) =  str2num(line(5:23));
   crc(i) = str2num(line(24:42));
   omega(i) = str2num(line(43:61));
   Omegadot(i) = str2num(line(62:80));
   line = fgetl(fide);	    %
   idot(i) = str2num(line(5:23));
   codes = str2num(line(24:42));
   weekno = str2num(line(43:61));
   L2flag = str2num(line(62:80));
   line = fgetl(fide);	    %
   svaccur = str2num(line(5:23));
   svhealth = str2num(line(24:42));
   tgd(i) = str2num(line(43:61));
   iodc = line(62:80);
   line = fgetl(fide);	    %
   tom(i) = str2num(line(5:23));
   spare = line(24:42);
%   spare = line(43:61);
%   spare = line(62:80);
end
status = fclose(fide);

%  Description of variable eph.
eph(1,:)  = svprn;
eph(2,:)  = af2;
eph(3,:)  = M0;
eph(4,:)  = roota;
eph(5,:)  = deltan;
eph(6,:)  = ecc;
eph(7,:)  = omega;
eph(8,:)  = cuc;
eph(9,:)  = cus;
eph(10,:) = crc;
eph(11,:) = crs;
eph(12,:) = i0;
eph(13,:) = idot;
eph(14,:) = cic;
eph(15,:) = cis;
eph(16,:) = Omega0;
eph(17,:) = Omegadot;
eph(18,:) = toe;
eph(19,:) = af0;
eph(20,:) = af1;
eph(21,:) = toe;

fidu = fopen(outputfile,'w');
count = fwrite(fidu,[eph],'double');
fclose all;
%%%%%%%%% end rinexe.m %%%%%%%%%
