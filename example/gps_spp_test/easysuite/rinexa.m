function rinexa(almanacfile, outputfile)
%RINEXA Reads a YUMA almanac and puts the data into
%            a matrix with 21 rows and a column for each satellite.
%            The matrix is stored in outputfile under the name eph

%Typical call: rinexa('current.alm','eph.dat')

%Kai Borre 13-02-16
%Copyright (c) by Kai Borre
%$Revision: 1.1 $  $Date: 2016/02/13  $

% Units are either seconds, meters, or radians

%almanacfile = 'current.alm';
%outputfile = 'eph.dat';
fide = fopen(almanacfile);
% The number of almanacs could be determined by counting all 
% lines in the almanac file devided by 15
noeph = 31; 

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
    line = fgetl(fide);
    line = fgetl(fide);
    svprn(i) = str2double(line(28:30));
    line = fgetl(fide);
    health(i) = str2double(line(28:31));
    line = fgetl(fide);
    ecc(i) = str2double(line(28:45));
    line = fgetl(fide);
    toe(i) = str2double(line(28:35));
    line = fgetl(fide);
    i0(i) = str2double(line(28:40));
    line = fgetl(fide);
    Omegadot(i) = str2double(line(28:45));
    line = fgetl(fide);
    roota(i) = str2double(line(28:39));
    line = fgetl(fide);
    Omega0(i) = str2double(line(28:45)); 
    line = fgetl(fide);
    omega(i) = str2double(line(28:39));
    line = fgetl(fide);
    M0(i) = str2double(line(28:45));
    line = fgetl(fide);
    af0(i) = str2double(line(28:45));
    line = fgetl(fide);
    af1(i) = str2double(line(28:45));
    line = fgetl(fide);
    weekno (i) = str2double(line(28:32));
    line = fgetl(fide); % reading a blank line
end;

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
count = fwrite(fidu,eph,'double');
fclose all;
%%%%%%%%% end rinexa.m %%%%%%%%%