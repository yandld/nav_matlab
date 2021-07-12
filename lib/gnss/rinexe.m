function iono = rinexe(ephemerisfile, outputfile)
%RINEXE Reads a RINEX Navigation Message file and
%	     reformats the data into a matrix with 23s
%	     rows and a column for each satellite.
%	     The matrix is stored in outputfile

%Typical call: rinexe('pta.96n','pta.nav')

%Kai Borre 04-18-96
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 1997/09/24  $

% Units are either seconds, meters, or radians
fide = fopen(ephemerisfile);
head_lines = 0;
iono = zeros(8,1);

%read the header
header_end = [];
while (isempty(header_end))
    head_lines = head_lines+1;
    %read the line and search the ionosphere labels
    lin = fgetl(fide);
    
    vers_found =  ~isempty(strfind(lin,'RINEX VERSION / TYPE'));
    iono_found = (~isempty(strfind(lin,'ION ALPHA')) || ~isempty(strfind(lin,'IONOSPHERIC CORR')));

    %if the ionosphere parameters label was found
    if (vers_found)
        version = str2num(lin(1:9));
    end
    
    %if the ionosphere parameters label was found
    if (iono_found)
        %change flag
        %         ioparam = 1;
        %save the 8 ionosphere parameters
        data = textscan(lin(5:end),'%f%f%f%f%*[^\n]');
        if ~isempty(data(4))
            iono(1) = data{1};
            iono(2) = data{2};
            iono(3) = data{3};
            iono(4) = data{4};
            lin = [];
            while isempty(lin)
                lin = fgetl(fide);
            end
            data = textscan(lin(5:end),'%f%f%f%f%*[^\n]');
            if ~isempty(data(4))
                iono(5) = data{1};
                iono(6) = data{2};
                iono(7) = data{3};
                iono(8) = data{4};
            else
                iono = zeros(8,1);
            end
        end
    end
    
    header_end = strfind(lin,'END OF HEADER');
end
head_lines = head_lines + 1;
noeph = -1; %初始化星历数目
while 1
   noeph = noeph+1;
   line = fgetl(fide);
   if line == -1, break;  end
end;
noeph = noeph/8  % 卫星星历数 = 去掉文件头之后的文件行数 / 8 
                 % 每颗卫星星历行数是8行

% 下面两行作用是回到文件头结束的地方
frewind(fide);
for i = 1:head_lines, line = fgetl(fide); end;

% Set aside memory for the input，
% 为星历中每个参数产生1行noeph列的0矩阵，
% 每一列对应该星历文件中的一个PRN
noeph = fix(noeph);
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
fit_interval = zeros(1,noeph);

% 将所有卫星的星历参数存入上面定义的矩阵中
for i = 1:noeph
   line = fgetl(fide);	  % 从该组星历中第一颗卫星的星历的第一行开始
   svprn(i) = str2num(line(1:2)); % 存入卫星PRN
   year = line(3:6);
   month = line(7:9);
   day = line(10:12);
   hour = line(13:15);
   minute = line(16:18);
   second = line(19:22);
   af0(i) = str2num(line(23:41));
   af1(i) = str2num(line(42:60));
   af2(i) = str2num(line(61:79));
   
   line = fgetl(fide);	  % 读下一行
   IODE = line(4:22);
   crs(i) = str2num(line(23:41));
   deltan(i) = str2num(line(42:60));
   M0(i) = str2num(line(61:79));
   
   line = fgetl(fide);	  % 读下一行
   cuc(i) = str2num(line(4:22));
   ecc(i) = str2num(line(23:41));
   cus(i) = str2num(line(42:60));
   roota(i) = str2num(line(61:79));
   
   line=fgetl(fide); % 读下一行
   toe(i) = str2num(line(4:22));
   cic(i) = str2num(line(23:41));
   Omega0(i) = str2num(line(42:60));
   cis(i) = str2num(line(61:79));
   
   line = fgetl(fide);	    % 读下一行
   i0(i) =  str2num(line(4:22));
   crc(i) = str2num(line(23:41));
   omega(i) = str2num(line(42:60));
   Omegadot(i) = str2num(line(61:79));
   
   line = fgetl(fide);	    % 读下一行
   idot(i) = str2num(line(4:22));
   codes = str2num(line(23:41));
   weekno = str2num(line(42:60));
   L2flag = str2num(line(61:79));
   
   line = fgetl(fide);	    % 读下一行
   svaccur = str2num(line(4:22));
   svhealth(i) = str2num(line(23:41));
   tgd(i) = str2num(line(42:60));
   iodc = line(61:79);
   
   line = fgetl(fide);	    % 读下一行
   if length(line)>= 41
       tom(i) = str2num(line(4:22));
       fit_interval(i) = str2num(line(23:41));
   else
       tom(i) = str2num(line(4:22));
       fit_interval(i) = 0;
   end
%    spare = line(42:60);
%    spare = line(61:79);
end
status = fclose(fide)

%  Description of variable eph.
% 把上面存好的各参数再转存到一个大矩阵“eph”中，该矩阵每行代表一种参数
% 每列代表一颗卫星
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
eph(22,:) = fit_interval;
eph(23,:) = svhealth;

fidu = fopen(outputfile,'w');
count = fwrite(fidu,[eph],'double');
fclose all
%%%%%%%%% end rinexe.m %%%%%%%%%
