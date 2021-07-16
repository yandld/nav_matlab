% EASY10     plot of ionospheric delays from phase observations.

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.1 $  $Date: 2005/01/20  $
%Total revision 2.0, January 9, 2016

% RINEX version3.03

% Initial computations of constants
v_light = 299792458;	     % vacuum speed of light m/s
f1 = 154*10.23E6;		     % L1 frequency Hz
f2 = 120*10.23E6;			 % L2 frequency Hz
lambda1 = v_light/f1;	     % wavelength on L1:  .19029367  m
lambda2 = v_light/f2;	     % wavelength on L2:  .244210213 m

% Read RINEX ephemerides file and convert to internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Selection of observation type
ss = 'C1W L1W C2W L2W' %; % pseudoranges and carrier phases

disp(' ')
disp('Now wait some time ...')

% Open the master observation file
ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

linjer = 0;
ij = [];
ant_delta = [];

% Gobbling the header, and collecting useful information
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
    
    answer = strfind(line,'ANT #');
    if ~isempty(answer)
        var = textscan(fid1,'%s %d14','Delimiter','\n');
        var1 = var{1}{1};
        ax = str2double(var1(1:14));
        ay = str2double(var1(15:28));
        az = str2double(var1(29:42));
    end
    
    answer = strfind(line,'SYS / # / OBS TYPES');
    if ~isempty(answer)
        tline1 = strsplit(line);
        line = fgetl(fid1);
        tline2 = strsplit(line);
        tt1 = horzcat(tline1,tline2);
        for qq = 1:4
            if qq == 1, i = strcmp(tt1,ss(1:3)); end
            if qq == 2, i = strcmp(tt1,ss(5:7)) ; end
            if qq == 3, i = strcmp(tt1,ss(9:11)); end
            if qq == 4, i = strcmp(tt1,ss(13:15)); end
            ii = find(i == 1) ;
            ii = ii-2;
            if ii > 15,  ii = ii-7; end;
            ij = [ij ii];
        end
        % the cell array of strings tline1 contains two strings in the start
        % which describe the system and number of observation types.
        % Both tline1 and tline2 termintes with six additional strings.
        % An extra string appears at  the start of tline2; it originates
        % from concatenation of the two lines. The indexing does not
        % change even if you empty the cells. They remain as empty
        % cells and keep a place
        obs_col = ij;
    end;
    answer = strfind(line,'INTERVAL');
    if ~isempty(answer)
        interval = strtok(line);
        int = str2double(interval);
    end;
end % end reading header

% the string arrays for the tline1 and tline2 contain an integer after the
% carrier phase observation. We must account for this by the following
% correctional table. The following code may be implemented more
% elegantly
for qq = 1:4
    if qq == 1, a0 = 6; b0 = 7; end
    if qq == 2, a0 = 6; b0 = 7; end
    if qq == 3, a0 = 14; b0 =15; end
    if qq == 4, a0 =14; b0 =15; end
    if     strcmp(ss(a0:b0), '1C') , obs_col(1,qq) = obs_col(1,qq) +1;
    elseif  strcmp(ss(a0:b0), '1W'), obs_col(1,qq) = obs_col(1,qq)+2;
    elseif strcmp(ss(a0:b0), '2X'), obs_col(1,qq) = obs_col(1,qq) +3;
    elseif strcmp(ss(a0:b0), '2W'), obs_col(1,qq) = obs_col(1,qq) +4;
    else  strcmp(ss(a0:b0), '5X'), obs_col(1,qq) = obs_col(1,qq) +5;
    end
end

Ion_fin = [];
epoch = 0;

while  ~feof(fid1)
    epoch = epoch +1;
    sats1 = [];
    
    % We use fgetl to read characters in the first line in every
    % epoch in the master file. Output is sow and number of SVs: NoSvs1.
    [time,post] = textscan(fid1,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    % static = str2double(tid(31:32));
    NoSvs1 = str2double(tid(34:36));
    dt1  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time1 = sec_of_week;
    
    Obs1 = zeros(NoSvs1,4);
    for j = 1:NoSvs1
        % we read the observations line by line. The first token identifies
        % the PRN. In this case the GPS 02 SV
        tline = fgetl(fid1);
        sats1(j,1)  = str2num(tline(2:3));
        as = strsplit(tline);
        % first and third col.s contain pseudorange, and second and
        % fourth col.s contain carrier phases
        for s = 1:4
            if size(as,2) < obs_col(s),  ass = NaN;
            else
                ass = as(obs_col(s));
            end
            Obs1(j,s) = str2double(ass);
        end
    end
    
    % Next we test if all observed SVs have an ephemeris.
    % sats1 contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the following manipulations
    Sats = intersect(sats1,Eph(1,:));
    
    %The command ismember does not change the sequence of entries in sats1
    lia = ismember(sats1,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats1(lia==0) = [];
    Obs1(lia==0,:) = []; % modified because of two columns
    
    % dimensions
    % time1  % 1 x 1
    % NoSV1  % 1 x 1
    % sats1 % NoSV1 x 1
    % Obs1 % NoSV1 x 4
    Ion(:, epoch) = (Obs1(:,4)*lambda2-Obs1(:,2)*lambda1)/(1-(f1/f2)^2);
    Ion_fin = [Ion_fin  Ion(:,epoch)-Ion(:,1)];
end

figure(1);
plot(Ion_fin','linewidth',2)
legend(eval('num2str(sats1)'),2)
title('Ionospheric Delay From {\itL}_1 and {\itL}_2 Phases','fontsize',16)
ylabel('Ionospheric Delay  [m]','fontsize',16)
xlabel('Epochs  [1 s interval]','fontsize',16)
set(gca,'fontsize',16)
legend

print -dpdf easy10

%%%%%%%%%%%%%%%%%%%% end easy10.m  %%%%%%%%%%%%%%%%%%%
