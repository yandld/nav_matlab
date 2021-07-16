% EASY19  Read RINEX navigation file reformat into Matlab Eph matrix.
%              Open a RINEX observation file, analyse the header and identify
%              observation types. Next we read the observations and use
%              recpo_ls to compute omc and a least-squares estimate for the
%              (stand alone) receiver position.

% Kai Borre 13-06-2008
% Copyright (c) by Kai Borre
% $Revision: 1.0 $  $Date: 2008/06/13  $
% Total revison 2.0, January 13, 2016

% RINEX version 3.03

global ME

set(0,'DefaultTextFontName','Times');
set(0,'DefaultAxesFontName','Times');
set(0,'DefaultTextFontSize',14);

v_light = 299792458;
dtr = pi/180;

rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Selection of observation type
ss = 'C1W'%;

% Open the observation file
ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

linjer = 0;
% Gobbling the header and collecting useful information
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
    
    answer = strfind(line,'SYS / # / OBS TYPES');
    if ~isempty(answer)
        tline1 = strsplit(line);
        line = fgetl(fid1);
        tline2 = strsplit(line);
        tt1 = horzcat(tline1,tline2);
        i = strcmp(tt1,ss); % if tt equals ss, i =1, else 0
        ii = find(i == 1) ;
        ii = ii-2;
        if ii > 15,  ii = ii-7; end;
        % the cell array of strings tline1 contains two strings in the start
        % which describe the system and number of observation types.
        % Both tline1 and tline2 termintes with six additional strings.
        % An extra string appears at  the start of tline2; it originates
        % from concatenation of  the two lines. The indexing does not
        % change even if you empty the cells. They remain as empty
        % cells and keep a place
        obs_col = ii;
    end;
    answer = strfind(line,'INTERVAL');
    if ~isempty(answer)
        interval = strtok(line);
        int = str2double(interval);
    end;
end % end reading header

% the string arrays for the tline1 and tline2 contain an integer after the
% carrier phase observation. We must account for this by the following
% correctional table
if        strcmp(ss(2:3), '1C'), obs_col = obs_col +1;
elseif  strcmp(ss(2:3), '1W'), obs_col = obs_col+2;
elseif strcmp(ss(2:3), '2X'), obs_col = obs_col +3;
else  strcmp(ss(2:3), '2W'), obs_col = obs_col +4;
end

Pos = [];
Gdop = [];
bases = [];
epoch = 0;

% This script needs information on the best possible
% coordinates of the receiver site.
% Often this information is furnished from another source.
% In our primitive code we estimate the receiver position
% as the average position of all epochs. It is called ME

while ~feof(fid1)
    epoch = epoch +1;
    sats = [];
    sats0 = [];
    
    % We read numerical data by fgetl and read the first line in every
    % epoch. It contains sow, number of SVs.
    [time,post] = textscan(fid1,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    static = str2double(tid(31:32));
    NoSvs = str2double(tid(34:36));
    dt  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time = sec_of_week; % sow1
    
    Obs = zeros(NoSvs,length(obs_col));
    for i = 1:NoSvs
        [obs1,~] = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1} ;
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1} ;
        sats(i,:) = str2double(sat1(2:3));
        Obs(i,:) = str2double(obs1y(obs_col));
    end
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the following manipulations
    Sats = intersect(sats,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats
    lia = ismember(sats,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats(lia==0) = [];
    Obs(lia==0) = [];
    % All book-keeping has prepared the data so that we can make the call
    % for a  final position computation
    pos = recpo_ls(Obs,sats,time,Eph);
    Pos = [Pos pos];
end % while
fclose(fid1);

ME = mean(Pos,2);
fprintf('\n\nMean Position as Computed From %d Epochs:', epoch)
fprintf('\n\nX: %12.3f  Y: %12.3f  Z: %12.3f\n\n', ME(1,1), ME(2,1), ME(3,1))

% Knowing a best possible receiver position we compute
% the differential corrections to the pseudoranges that have
% to be transmitted to the possible rover receivers.

fid1 = fopen(ofile1,'rt');

linjer = 0;
% Gobbling the header and collecting useful information
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
end % end reading header

dt1 = [];

while  ~feof(fid1)
    epoch = epoch +1;
    sats1 = [];
    % Reading first observation record in master file.
    
    % We read numerical data by fgetl. We read the first line in every
    % epoch in the master file. Output is sow and number of SVs: NoSvs1.
    [time,post] = textscan(fid1,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    static = str2double(tid(31:32));
    NoSvs1 = str2double(tid(34:36));
    dt1  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time1 = sec_of_week; % sow1
    
    Obs1 = zeros(NoSvs1,length(obs_col));
    for i = 1:NoSvs1
        [obs1,~] = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1} ;
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1} ;
        sats1(i,:) = str2double(sat1(2:3));
        Obs1(i,:) = str2double(obs1y(obs_col));
    end
    
    % Next we test if all observed SVs have an ephemeris.
    % sats1 contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the following manipulations
    Sats = intersect(sats1,Eph(1,:));
    
    % The command ismember does not change the sequence
    % of entries in sats1
    lia = ismember(sats1,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted.
    % We delete the corresponding row in the observations
    sats1(lia==0) = [];
    Obs1(lia==0) = [];
    NoSV1 = length(sats1);
end % first reading of observation file

% we close all open files and re-open for reading
% from the beginning
fclose(fid1);

ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

linjer = 0;
% Gobbling the header once more
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
end

Omc = [];
epoch = 0;
dt = [];
Tline = [];

while ~feof(fid1)
    epoch = epoch +1;
    sats = [];
    
    % We read numerical data by fgetl and read the first line in every
    % epoch. It contains sow, number of SVs.
    [time,post] = textscan(fid1,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    static = str2double(tid(31:32));
    NoSvs = str2double(tid(34:36));
    dt = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time = sec_of_week; % sow1
    
    Obs = zeros(NoSvs,length(obs_col));
    for i = 1:NoSvs
        [obs1,~] = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1} ;
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1} ;
        sats(i,:) = str2double(sat1(2:3));
        Obs(i,:) = str2double(obs1y(obs_col));
    end
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the following manipulations
    Sats = intersect(sats,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats
    lia = ismember(sats,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats(lia==0) = [];
    Obs(lia==0) = [];
    % All book-keeping has prepared the data so that we can nake the call
    % for a  final position computation
    [pos, ~, ~,omc]  = recpo_ls(Obs,sats,time,Eph);
    Omc = [Omc omc];
end % while
fclose(fid1);

figure(1);
subplot(2,1,1), plot(Omc','linewidth',.2)
set(gca,'Fontsize',16);
xlabel('Epochs [1 s]')
ylabel({'Range'; 'Correction [m]'})

subplot(2,1,2), plot(diff(Omc,1,2)','linewidth',.3)
set(gca,'Fontsize',16);
xlabel('Epochs [1 s]')
ylabel({'Range Rate'; 'Correction [m/s]'})

print -dpdf easy19

%%%%%%%%%%%%%%%%%%%%% end easy19.m %%%%%%%%%%%%%%%