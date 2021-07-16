%EASY4     The ephemeris file is reformated by rinexe. The observation type
%               is selected and the observation files for master and rover are opened.
%               The preliminary (X,Y,Z) position of the master receiver is read from
%               the header.
%               The reading position in the file is moved to the first epoch and
%               sow is computed. sow in master and rover files are synchronized.
%               We test if all SVs have an ephemris and SVs and their
%               observations are sorted to match. Next we call the function
%               baseline which computes the components of the baseline.
%              Some results are printed in workspace and some figures are plotted.

% The code could be shortened by introducing functions. However, for a
% teaching situation I found it more illustrative to compile a script.

%RINEX version 3.03

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/07/27  $
% Total revison 2.0, January 13, 2016

% Read RINEX ephemerides file and convert to internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Selection of observation type
ss = 'C1W'%;

% Open the master observation file
ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

% Open the rover observation file
ofile2 = 'log_r.15o'; % log_24h.15o
fid2 = fopen(ofile2,'rt');

coo = [];
ant_delta = [];
linjer = 0;

% Gobbling the master header, and collecting useful information
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
    
    answer = strfind(line,'REC #');
    if ~isempty(answer)
        var = textscan(fid1,'%s %d14','Delimiter','\n');
        var1 = var{1}{1};
        cox = str2double(var1(1:14));
        coy = str2double(var1(15:28));
        coz = str2double(var1(29:42));
    end
    
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
        i = strcmp(tt1,ss); % if tt equals ss, i =1, else 0
        ii = find(i == 1) ;
        ii = ii-2;
        if ii > 15,  ii = ii-7, end;
        % the cell array of strings tline1 originally contains two strings
        % which describe the system and number of observation types.
        % Both tline1 and tline2 terminates with six additional strings.
        % An extra string appears at  the start of tline2; it originates
        % from concatenation of the two lines. The indexing does not
        % change even if you empty the cells. They remain as empty
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
% carrier phase observation. We must account for this by the following
% correctional table
if         strcmp(ss(2:3), '1C'), obs_col = obs_col +1;
elseif  strcmp(ss(2:3), '1W'), obs_col = obs_col+2;
elseif strcmp(ss(2:3), '2X'), obs_col = obs_col +3;
else  strcmp(ss(2:3), '2W'), obs_col = obs_col +4;
end

Pos = [];
dt1 = [];
dt2 = [];
Gdop = [];
Omc = [];
bases = [];
epoch = 0;

while  1
    epoch = epoch +1;
    time1 = 0;
    sats1 = [];
    time2 = 0;
    sats2 = [];
    
    % Reading first observation record in master file.
    
    %  We read the first line in every epoch in the master file.
    % Output is sow nd number of SVs: NoSvs1.
    [time,~] = textscan(fid1,'%s %d8','Delimiter','\n');
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
    time1 = sec_of_week; % sow1
    
    Obs1 = zeros(NoSvs1,length(obs_col));
    % Reading NoSvs1 lines of observations in the master file
    for i = 1:NoSvs1
        [obs1,~] = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1} ;
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1} ;
        sats1(i) = str2double(sat1(2:3));
        Obs1(i,:) = str2double(obs1y(obs_col));
    end
    
    % Next we test if all observed SVs have an ephemeris.
    % sats1 contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the follwing manipulations
    Sats = intersect(sats1,Eph(1,:));
    
    %The command ismember does not change the sequence of entries in sats1
    lia = ismember(sats1,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats1(lia==0) = [];
    Obs1(lia==0) = [];
    NoSV1 = length(sats1);
    
    % dimensions
    % time1  % 1 x 1
    % NoSV1  % 1 x 1
    % sats1 % NoSV1 x 1
    % Obs1 % NoSV1 x 1
    
    % the whole story repated for the rover with index 2
    % we start by gobbeling the header
    if epoch == 1
        for qq = 1:linjer+ 3
            fgetl(fid2);
        end
    end
    
    % We read the header line for the first epoch in the rover file.
    % It gives us sow and number of SVs.
    [time,~] = textscan(fid2,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    % static = str2double(tid(31:32));
    NoSvs2 = str2double(tid(34:36));
    dt2  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time2 = sec_of_week; % sow2
    
    % Reading the observations in the first epch in the rover file
    Obs2 = zeros(NoSvs2,length(obs_col));
    for i = 1:NoSvs2
        [obs2,~] = textscan(fid2,'%s %d8','Delimiter','\n');
        obs2y = obs2{1}{1};
        obs2 = strsplit(obs2y);
        sat2 = obs2{1};
        sats2(i,:) = str2double(sat2(2:3));
        Obs2(i,:) = str2double(obs2(obs_col));
    end
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the follwing manipulations
    Sats = intersect(sats2,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats
    lia = ismember(sats2,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats2(lia==0) = [];
    Obs2(lia==0) = [];
    NoSV2 = length(sats2);
    
    % dimensions
    % time2  % 1 x 1
    % NoSV2  % 1 x 1
    % sats2   % NoSV2 x 1
    % Obs2  % NoSV2 x  1
    
    if epoch == 1
        % Establish synchronous reading of fid1 and fid2
        % The epoch interval is int seconds      
        if  time1 > time2
            for i = 1: round(time1-time2)
                 [time,~] = textscan(fid2,'%s %d8','Delimiter','\n');
                tid = time{1}{1};
                year = str2double(tid(3:6));
                month = str2double(tid(8:9));
                day = str2double(tid(11:12));
                hour = str2double(tid(14:15));
                minute = str2double(tid(17:18));
                second = str2double(tid(20:29));
                %static = str2double(tid(31:32));
                NoSvs2 = str2double(tid(34:36));
                dt2  = str2double(tid(38:56));
                h = hour+minute/60+second/3600;
                jd = julday(year, month, day, h);
                [~, sec_of_week] = gps_time(jd);
                time2 = sec_of_week; % sow2
                
                for qq = 1:NoSvs2
                    fgetl(fid2);
                end
            end % i
        end % if time1 ...
              
        if  time1 < time2
            for i = 1: round(time2-time1)  
                [time, ~] = textscan(fid1,'%s %d8','Delimiter','\n');
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
                
                for qq = 1:NoSvs1
                    fgetl(fid1);
                end
            end % i
        end % if time1
        fprintf('First Common Epoch Time1 :  %8.0f\n', time1)
        fprintf('First Common Epoch Time2 :  %8.0f\n', time2)
    end % synchronization
    
    % By chance sats1 and sats 2 are the same. Otherwise we need to establish
    % a matching between sats1 and sats2 and likewise for Obs1 and Obs2
    
    % master observations: obs1, and rover observations: obs2
    [omc,base] = baseline([cox;coy;coz],Obs1,Obs2,sats1,time1,Eph);
    Omc = [Omc, omc];
    bases = [bases base];
    
    [pos, el, gdop] = recpo_ls(Obs1,sats1,time1,Eph);
    Gdop = [Gdop gdop];
    Pos = [Pos pos];
    if feof(fid1) == 1, break, end;
    if feof(fid2) == 1, break; end;
end % while
fclose all;

me1 = mean(bases,2);
spread1 = std(bases,1,2);
fprintf('\nBaseline Components as Computed From %2.0f Epochs:', epoch)
fprintf('\n\nX: %12.3f  Y: %12.3f  Z: %12.3f\n\n', me1(1,1),me1(2,1),me1(3,1))

figure(1);
plot((bases(:,2:end)-bases(:,2)*ones(1,epoch-1))','linewidth',.25)
title(['Variation of Baseline Components Over ',int2str(epoch),' Epochs'],'fontsize',14)
legend('X','Y','Z')
xlabel(['Epochs ', int2str(int), ' s interval'], 'fontsize',14)
ylabel('[m]','fontsize',14)
set(gca,'fontsize',14)
legend
print -dpdf easy41

figure(2);
plot(Omc(:,2:end)')
xlabel('Epochs')
ylabel('[m]','fontsize',14)
title({'Observed minus Computed Values'; 'for All SVs in Each Epoch'},'fontsize',14)
set(gca,'fontsize',14)
print -dpdf easy42

figure(3);
plot(Gdop,'linewidth',1)
axis([1 length(Gdop) 0 5])
title('GDOP')
print -dpdf easy43

%%%%%%%%%%%%%% end easy4.m %%%%%%%%%%%%%%%

