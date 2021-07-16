% EASY8   Test for cycle slip  after idea by Kees de Jong.

%Kai Borre 26-12-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/12/26  $
%Total revision 2.0, February 14, 2016

% RINEX version3.03

% Reference:
%  Kees de Jong (1998):  Real-time integrity monitoring, ambiguity
%  resolution and kinematic positioning with GPS. Proceedings of 2nd European
%  Symposium on GNSS '98, pages VIII7/1--VIII7/7, Toulouse, France

% Initial computations of constants
v_light = 299792458;	     % vacuum speed of light m/s
f1 = 154*10.23E6;		     % L1 frequency Hz
f2 = 120*10.23E6;			 % L2 frequency Hz
lambda1 = v_light/f1;	     % wavelength on L1:  .19029367  m
lambda2 = v_light/f2;	     % wavelength on L2:  .244210213 m
alpha = (f1/f2)^2;

% Read RINEX ephemerides file and convert to internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Selection of observation type
%ss = 'C1W L1W C2W L2W' %; % pseudorange and carrier phase
ss = 'C1C L1C C2X L2X' %; % just to test other obsrervation types
% Open the master observation file
ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

% Open the rover observation file
ofile2 = 'log_r.15o';
fid2 = fopen(ofile2,'rt');

coo = [];
ant_delta = [];
linjer = 0;
ij = [];

fighdl1 = figure(1);
h1 = animatedline('color','r');
h2 = animatedline('color','b');
h3 = animatedline('color','g');
h4 = animatedline('color','k');
axis([0 900 -3 2]);

% Gobbling the master file header, and collecting useful information
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
        % from concatenation of  the two lines. The indexing does not
        % change even if you empty the cells. They remain as empty
        % cells and keep a place
        obs_col = ij;
    end;
    answer = strfind(line,'INTERVAL');
    if ~isempty(answer)
        interval = strtok(line);
        int = str2double(interval);
    end;
    answer = strfind(line,'# OF SATELLITES');
    if ~isempty(answer)
        NumberSV = strtok(line);
        numSV = str2double(NumberSV)% ;
    end
end % end reading header

% the string arrays for the tline1 and tline2 contain an integer after the
% carrier phase observation. We must account for this by the following
% correctional table. To avoid applying the correction twice, we add
% breaks

% the following code may be done more elegantly
for qq = 1:4
    if qq == 1, a0 = 6; b0 = 7; end
    if qq == 2, a0 = 6; b0 = 7; end
    if qq == 3, a0 = 14; b0 =15; end
    if qq == 4, a0 =14; b0 =15; end
    if         strcmp(ss(a0:b0), '1C'), obs_col(1,qq) = obs_col(1,qq) +1;
    elseif  strcmp(ss(a0:b0), '1W'), obs_col(1,qq) = obs_col(1,qq)+2;
    elseif  strcmp(ss(a0:b0), '2X'), obs_col(1,qq) = obs_col(1,qq) +3;
    elseif  strcmp(ss(a0:b0), '2W'), obs_col(1,qq) = obs_col(1,qq) +4;
    else    strcmp(ss(a0:b0), '5X'), obs_col(1,qq) = obs_col(1,qq) +5; % == '5X'
    end
end

epoch = 0;
OBS1 = [];
OBS2 = [];

% Preparations for filter
x = zeros(4,1);      % state vector: [B1 B2 B3 Idot]
delta_t = 1;         % epoch interval in seconds
F = eye(4);
F(1:3,4) = delta_t;
A = diag([alpha-1, -2, -alpha-1]);
A = [A zeros(3,1)];
T = [-ones(3,1) eye(3)];
Sigma_b = 2*diag([0.3^2 0.3^2 0.003^2 0.003^2]);
Sigma_e = T*Sigma_b*T';
Sigma_epsilon = diag([.1^2 .1^2 .1^2 .1^2]);
P = 10^2*eye(4);
X = [];

while  1
    epoch = epoch +1;
    time1 = 0;
    sats1 = [];
    time2 = 0;
    sats2 = [];
    
    % Reading first observation record in master file.
    
    % We use fgetl to read charactes in the first line in every
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
    
    Obs1 = zeros(NoSvs1,4);
    for j = 1:NoSvs1
        % we read the observations line by line. The first token identifies
        % the PRN, In this case the GPS 02 SV
        tline = fgetl(fid1);
        sats1(j,1)  = str2num(tline(2:3));
        as = strsplit(tline);
        % first and third col.s are pseudorange and second and
        %thrid col.s are code
        for s = 1:4
            if size(as,2) < obs_col(s),  ass = NaN;
            else
                ass = as(obs_col(s));
            end
            Obs1(j,s) = str2double(ass);
        end
    end
    
    OBS1 = [OBS1 Obs1];
    
    % Reading first observation record in rover file
    
    if epoch == 1
        % the whole story repated for the rover with index 2
        % we start by gobbeling the header
        for qq = 1:linjer+3
            fgetl(fid2);
        end
        % Selection of Sv to be tested
        disp(' ')
        sst = input('Select Number of SV (2, 5, 6, 31, 24, 25, 12, 29): ');
        if sst ~= [2 5 6 31 24 25 12 29], break, end
        kk = find(sats1 == sst);
    end
    
    % We use fgetl, which reads characters, to read the first line in every
    %epoch. It contains sow, number of SVs.
    [time,pos] = textscan(fid2,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    static = str2double(tid(31:32));
    NoSvs2 = str2double(tid(34:36));
    dt2  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time2 = sec_of_week; % sow2
    
    Obs2 = zeros(NoSvs2,4);
    for j = 1:NoSvs2
        % we read the observations line by line. The first token identifies
        % the PRN, In this case the GPS 02 SV
        tline = fgetl(fid2);
        sats2(j,1)  = str2double(tline(2:3));
        as = strsplit(tline);
        % first and third  col.s contain pseudoranges and second and
        %fourth col.s contain carrier phases
        
        % the following fills in NaN.s in case of missing data
        for s = 1:4
            if size(as,2) < obs_col(s), ass = NaN;
            else
                ass = as(obs_col(s));
            end
            Obs2(j,s) = str2double(ass);
        end
    end
    
    if epoch == 1
        % Establish synchronous reading of fid1 and fid2
        % The epoch interval is int seconds
        if  time1 > time2
            for i = 1: round(time1-time2)
                [time,pos] = textscan(fid2,'%s %d8','Delimiter','\n');
                tid = time{1}{1};
                year = str2double(tid(3:6));
                month = str2double(tid(8:9));
                day = str2double(tid(11:12));
                hour = str2double(tid(14:15));
                minute = str2double(tid(17:18));
                second = str2double(tid(20:29));
                static = str2double(tid(31:32));
                NoSvs2 = str2double(tid(34:36));
                dt2  = str2double(tid(38:56));
                h = hour+minute/60+second/3600;
                jd = julday(year, month, day, h);
                [~, sec_of_week] = gps_time(jd);
                time2 = sec_of_week; % sow2
                
                for qq = 1:NoSvs2
                    fgetl(fid2);
                end
            end
        end % new
        
        if  time1 < time2
            for i = 1: round(time2-time1)
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
                       
                for qq = 1:NoSvs1
                    fgetl(fid1);
                end
            end
        end
        
        fprintf('\nFirst Common Epoch Time1 :  %8.0f', time1)
        fprintf('\nFirst Common Epoch Time2 :  %8.0f\n', time2)
    end % synchronization
    
    OBS2 = [OBS2 Obs2];
    
    % By chance sats1 and sats 2 are the same in the sample data. Otherwise
    % we need to establish  a matching between sats1 and sats2 and
    % likewise for Obs1 and Obs2
    
    % right side for master receiver
    bm = [Obs1(:,3)-Obs1(:,1);                  % P2-P1
        Obs1(:,2)*lambda1-Obs1(:,1);   % Phi1-P1
        Obs1(:,4)*lambda2-Obs1(:,1)];  % Phi2-P1
    
    % right side for rover receiver
    br = [Obs2(:,3)-Obs2(:,1);                  % P2-P1
        Obs2(:,2)*lambda1-Obs2(:,1);   % Phi1-P1
        Obs2(:,4)*lambda2-Obs2(:,1)];  % Phi2-P1
    
    b = bm(s)-br(s);
    % Kalman filter
    x = F*x;
    P = F*P*F'+Sigma_epsilon;
    K = P*A'*inv(A*P*A' + Sigma_e);
    x = x+K*(b-A*x);
    P = (eye(4)-K*A)*P;
    
    addpoints(h1,epoch, x(1))
    addpoints(h2,epoch, x(2))
    addpoints(h3,epoch,x(3))
    addpoints(h4,epoch,x(4))
    drawnow
    
    if feof(fid1), break, end
    if feof(fid2), break, end
    X = [X x];
end % while

fighdl2 = figure(2);
plot(1:epoch-1,X','linewidth',1)
title(['Filter result for PRN  ' num2str(sats1(kk))],'fontsize',14)
ylabel('Variations in {\itB}_1, {\itB}_2, {\itB}_3, and {\itdI/dt}  [m]','fontsize',14)
xlabel('Epochs [1 s interval]','fontsize',14)
legend('{\itB}_1','{\itB}_2','{\itB}_3','{\itdI/dt}')
set(gca,'fontsize',14)
legend

print -dpdf easy82

fighdl3 = figure(3);
plot(1:epoch-2,diff(X(2:3,:),1,2)','linewidth',1)
title(['Filtered pseudoranges for PRN  ' num2str(sats1(kk))],'fontsize',14)
ylabel('Differences from epoch to epoch [m]','fontsize',14)
xlabel({'Epochs [1 s interval]', ['A cycle slip occured if ',  ....
    '|{\itB}_2| > {\lambda}_1 or |{\itB}_3| > {\lambda_2}']},'fontsize',14)
legend('{\itB}_2','{\itB}_3')
set(gca,'fontsize',14)
legend

print -dpdf easy83

return

% Repair of clock reset of 1ms ~ 299 km; affects only pseudoranges
i1 = find(abs(DP(1,:)) > 280000);

for j = i1
    if DP(:,j) < 0
        DP(:,j) = DP(:,j)+299792.458;
    else
        DP(:,j) = DP(:,j)-299792.458;
    end
end

figure(4);
plot((deltaP-deltaPhi)','linewidth',2)
legend(eval('num2str(sv)'),2)
title('Check of Cycle Slips')
ylabel('Misclosure [m]')
xlabel('Epochs  [Interval 1 s]')
legend

%%%%%%%%% end easy8.m %%%%%%%%%