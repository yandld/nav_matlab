% EASY3	  Read RINEX navigation file, version 3.03, and reformat
%             into a Matlab Eph matrix. Open a RINEX observation file,
%             version 3.03, analyse the header and identify observation
%             types. The call fgetl finds the information for the epoch time.
%             Next we read observations line  by line. Finally recpo_ls
%             estimates the  (stand alone) receiver position.

% Kai Borre 31-10-2001
% Copyright (c) by Kai Borre
% $Revision: 1.0 $  $Date: 2001/10/31  $
% Total revision 2.0, February 13, 2016

% RINEX version 3.03


% Read RINEX ephemerides file and convert to internal Matlab format
rinexe('log_24h.15n','eph.dat');
% rinexe('D:\Work\Laboratory\Trana\rec\r302_long\Bas_log_2016_05_07_12.00.00.16N','eph.dat');
Eph = get_eph('eph.dat');

% Open the observation file
ofile2 = 'log_r.15o'; % log_24h.15o
% ofile2 = 'D:\Work\Laboratory\Trana\rec\r302_long\Bas_log_2016_05_07_12.00.00.16o';
fid2 = fopen(ofile2,'rt');

% The selection of observation type is set
ss = 'C1W'%;
linjer = 0;

while 1			   % Gobbling the header
    linjer = linjer +1;
    line = fgetl(fid2);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
    answer = strfind(line,'ANT # / TYPE');
    if ~isempty(answer)
        delta = textscan(fid2,'%6.4f','Delimiter','\n');
        delt = delta{1};
        delx = delt(1,1);
        dely = delt(2,1);
        delz = delt(3,1);
    end
    
    answer = strfind(line,'SYS / # / OBS TYPES');
    if ~isempty(answer)
        tline1 = strsplit(line);
        line = fgetl(fid2);
        tline2 = strsplit(line);
        tt = horzcat(tline1,tline2);
        i = strcmp(tt,ss); % if tt equals ss, i =1, else 0
        ii = find(i == 1) ;
        ii = ii-2;
        if ii > 15,  ii = ii-7; end;
        % the cell array of strings tline1 originally contains two strings
        % which describe the system and number of observation types.
        % Both tline1 and tline2 terminates with six additional strings.
        % An extra string appears at  the start of tline2; it originates
        % from concatenation of  the two lines. The indexing does not
        % change, even if you empty the cells. They remain empty
        % cells and keep a place
        obs_col = ii%;
    end;
    answer = strfind(line,'INTERVAL');
    if ~isempty(answer)
        interval = strtok(line);
        int = str2double(interval);
    end;
end % end reading header

% the string arrays for the tline1 and tline2 contain an integer after the
% carrier phase observation. We account for this by the following
% correctional table
if         strcmp(ss(2:3), '1C'), obs_col = obs_col +1;
elseif   strcmp(ss(2:3), '1W'), obs_col = obs_col+2;
elseif  strcmp(ss(2:3), '2X'), obs_col = obs_col +3;
else    strcmp(ss(2:3), '2W'), obs_col = obs_col +4;
end

Pos = [];
epoch = 0;
dt = [];
Tline = [];

while ~feof(fid2)
    epoch = epoch +1;
    %time = 0;
    sats = [];
    sats0 = [];
    
    % We read the first line in every  epoch and get sow and
    % number of SVs.
    [time,post] = textscan(fid2,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    static = str2double(tid(31:32));
    NoSvs = str2double(tid(34:36));
    dte  = str2double(tid(38:56));
    dt = [dt dte];
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time = sec_of_week; % sow
    
    Obs = zeros(NoSvs,length(obs_col));
    for i = 1:NoSvs
        obs = textscan(fid2,'%s %d8','Delimiter','\n');
        obsy = obs{1}{1};
        obs = strsplit(obsy);
        sat = obs{1};
        sats(i,:) = str2double(sat(2:3));
        Obs(i,1) = str2double(obs(obs_col));
    end
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the follwing manipulations
    Sats = intersect(sats,Eph(1,:));
    
    %The command ismember does not change the sequence of entries in sats
    lia = ismember(sats,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats(lia==0) = [];
    Obs(lia==0) = [];
    % All book-keeping has prepared the data so that we can nake the call
    % for a  final position computation
    pos = recpo_ls(Obs,sats,time,Eph);
    Pos = [Pos pos];
end % while
fclose(fid2);

me = mean(Pos,2);
fprintf('\n\nMean Position as Computed From %d Epochs:', epoch)
fprintf('\n\nX: %12.3f  Y: %12.3f  Z: %12.3f\n\n', me(1,1), me(2,1), me(3,1))

figure(1);
plot(1:epoch,(Pos(1,:)-Pos(1,1)*ones(1,epoch))','-',...
    1:epoch,(Pos(2,:)-Pos(2,1)*ones(1,epoch))','-.',...
    1:epoch,(Pos(3,:)-Pos(3,1)*ones(1,epoch))','--','linewidth',.25)
title('Positions over time','fontsize',16)
legend('X','Y','Z')
xlabel('Epochs [1 s interval]','fontsize',16)
ylabel('Changes in {\itX}, {\itY}, {\itZ} since the first epoch [m]','fontsize',16)
set(gca,'fontsize',16)
legend
print -dpdf easy31

figure(2);
plot(1:epoch,dt*10^6')
ylabel('Receiver clock off-set in {\mu}s','fontsize',16),
xlabel('Epochs [1 s interval]','fontsize',16)
print -dpdf  easy32

% transformation from (X, Y, Z) to geographical coordinates
[phi, lambda, h] = cart2geo(me(1,1), me(2,1), me(3,1), 5)
%%%%%%%%%%%%%%%%%%%%% end easy3.m %%%%%%%%%%%%%%%



