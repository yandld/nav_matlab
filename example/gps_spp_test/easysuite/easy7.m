% EASY7 Receiver clock offset estimated in three ways:
%           1. as read from the RINEX file,
%           2. as computed via the position solution, and
%           3. by using Gaussian elimination on the normals

%Kai Borre 27-07-2002
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2002/07/27  $
% Total revision, February 16, 2016

% RINEX version 3.03


v_light = 299792458;	     % vacuum speed of light [m/s]

% Read RINEX ephemerides file and store in internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Open the observation file
ofile2 = 'log_24h.15o';
fid2 = fopen(ofile2,'rt');

% We select the observation type
ss = 'C1W'%;

pos = [];
ant_delta = [];
linjer = 0;

while 1			   % Gobbling the header
    linjer = linjer +1;
    line = fgetl(fid2);
    
    answer = strfind(line,'APPROX POSITION XYZ');
    if ~isempty(answer)
        for k = 1:3
            [X0, line] = strtok(line);
            po = str2double(X0);
            pos = [pos; po];
        end;
    end
    
    answer = strfind(line,'ANTENNA: DELTA H/E/N');
    if ~isempty(answer)
        for k = 1:3
            [delta, line] = strtok(line);
            del = str2double(delta);
            ant_delta = [ant_delta del];
        end;
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
        if ii > 15,  ii = ii-7, end;
        % the cell array of strings tline1 contains two strings in the start
        % which descrirbe the system and number of observation types.
        % Both tline1 and tline2 terminates with six additional strings.
        % An extra string appears at the start of tline2; it originates
        % from concatenation of the two lines. The indexing does not
        % change even if you empty the cells. They remain as empty
        % cells and keep a place
        obs_col = ii;
    end;
    
    answer = strfind(line,'INTERVAL');
    if ~isempty(answer)
        interval = strtok(line);
        int = str2double(interval);
    end;
    
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    if (line == -1), eof = 1; break; end;
    
end % reading header

% the string arrays for tline1 and tline2 contain an integer after the
% carrier phase observation. We must account for this by the following
% correctional table
if         strcmp(ss(2:3), '1C'), obs_col = obs_col +1;
elseif  strcmp(ss(2:3), '1W'), obs_col = obs_col+2;
elseif strcmp(ss(2:3), '2X'), obs_col = obs_col +3;
else  strcmp(ss(2:3), '2W'), obs_col = obs_col +4;
end

Pos = [];
epoch = 0;
dT = [];
Tline = [];
eTe = [];
eTb = [];
eTA = [];

while ~feof(fid2)
    epoch = epoch +1;
    time = 0;
    sats = [];
    sats0 = [];
    
    % We read numerical data by fgetl and read the first line in every
    % epoch. It contains sow, number of SVs.
    
    [time,post] = textscan(fid2,'%s %d8','Delimiter','\n');
    tid = time{1}{1};
    year = str2double(tid(3:6));
    month = str2double(tid(8:9));
    day = str2double(tid(11:12));
    hour = str2double(tid(14:15));
    minute = str2double(tid(17:18));
    second = str2double(tid(20:29));
    % static = str2double(tid(31:32));
    NoSvs = str2double(tid(34:36));
    dt  = str2double(tid(38:56));
    h = hour+minute/60+second/3600;
    jd = julday(year, month, day, h);
    [~, sec_of_week] = gps_time(jd);
    time = sec_of_week;
    dT = [dT dt];
    
    for j = 1:NoSvs
        % we read the observations line by line. The first token identifies
        % the PRN, In this case the GPS 02 SV
        tline = fgetl(fid2);
        sats(j,1)  = str2num(tline(2:3));
        for jj = 1: obs_col
            [xx,tline] = strtok(tline);
        end
        Obs(j,1) = str2double(xx);
    end
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the follwing manipulations
    Sats = intersect(sats,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats
    lia = ismember(sats,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats(lia==0) = [];
    Obs(lia==0) = [];
    
    % Finding columns in Eph for each SV
    NoSvs = length(sats);
    for t = 1:NoSvs
        col_Eph(t) = find_eph(Eph,sats(t),time);
    end
    
    reduced_normals = zeros(3,3);
    reduced_absolute = zeros(3,1);
    no_epochs = 0;
    tr_RAW = time;
    % Formation of Observation Equations
    A = zeros(NoSvs,3);
    omc = zeros(NoSvs,1);
    
    for iteration = 1:3
        for j = 1:NoSvs
            k = col_Eph(j);
            tx_RAW = tr_RAW - Obs(j)/v_light;
            Toc = Eph(21,k);
            dt = check_t(tx_RAW - Toc);
            a0 = Eph(19,k);
            a1 = Eph(20,k);
            a2 = Eph(2,k);
            tcorr = a0 + (a1 + a2*dt)*dt;
            tx_GPS = tx_RAW - tcorr;
            X = satpos(tx_GPS, Eph(:,k));
            traveltime = 70.e-3;	  % 70 ms first guess
            for iter = 1:2
                Rot_X = e_r_corr(traveltime, X);
                rho = norm(Rot_X - pos(1:3,1));
                traveltime = rho/v_light;
            end; % iter-loop
            [phi,lambda,h] = togeod(6378137, 298.257223563, ...
                pos(1,1), pos(2,1), pos(3,1));
            [az,el,dist] = topocent(Rot_X, Rot_X-pos(1:3,1));
            corrected_pseudorange = Obs(j) - ...
                tropo(sin(el),h/1000,1013.0,293.0,50.0,0.0,0.0,0.0);
            dx = Rot_X(1) - pos(1,1);
            dy = Rot_X(2) - pos(2,1);
            dz = Rot_X(3) - pos(3,1);
            distance = norm([dx dy dz]);
            calculated_pseudorange = distance - v_light*tcorr;
            omc(j,1) = corrected_pseudorange - calculated_pseudorange;
            A(j,1) = -dx/distance;
            A(j,2) = -dy/distance;
            A(j,3) = -dz/distance;
        end; % j loop
    end % iteration
    
    % Formation of Normal Equations
    % We have NoSvs number of sv.s
    %		    b = omc        right side,  dimension NoSvs by 1;
    %		    A			        dimension NoSvs by 3;
    %		    sum(A)		    dimension 1 by NoSvs;
    eTe = [eTe NoSvs];
    eTb = [eTb sum(omc)];
    eTA = [eTA sum(A)'];
    reduced_normals = reduced_normals + A'*A-sum(A)'*sum(A)/NoSvs;
    reduced_absolute = reduced_absolute + ...
        A'*omc - sum(A)'*sum(omc)/NoSvs;
    x = inv(reduced_normals)*reduced_absolute;
    % All book-keeping has prepared the data so that we can make the call
    % for a  final position computation
    pos = recpo_ls(Obs,sats,time,Eph); % pos(4,1) is in [m]
    Pos = [Pos pos];
end % while loop over epochs

fclose('all');

me = mean(Pos,2);
fprintf('\n\nMean Position as Computed From %d Epochs:', epoch)
fprintf('\n\nX: %12.3f  Y: %12.3f  Z: %12.3f\n\n', me(1,1), me(2,1), me(3,1))
% transformation from (X, Y, Z) to geographical coordinates
[phi, lambda, h] = cart2geo(me(1,1), me(2,1), me(3,1), 5);
phi
lambda
h

figure(1);
hold on
plot(1:size(dT,2), 1000*dT','-.r', 'linewidth',4)  % time in [ms]
plot(1000*Pos(4,:)/v_light)                              % time in [ms]
% The dT values are read from the observation file while the Pos(4,:)
% values originate from the position computation. The difference between
% the two graphs is some 30 ns which of course is not visible in the
% figure.
hold off
ylabel('Receiver Clock Off-set [ms]','fontsize',16),
xlabel('Epochs [1 s interval]','fontsize',16)
set(gca,'Fontsize',16)

print -dpdf  easy71

disp('The thick dashed line in Figure 1 represents the receiver clock-offset')
disp('as read in the RINEX observation file;')
disp('the thin solid line underneath is the offset as computed on')
disp('an epoch-by- epoch basis')

rec_clk_offset = zeros(1,size(dT,2));
for m = 1:size(dT,2)
    rec_clk_offset(1,m) =  (eTb(m)-eTA(:,m)'*x)/(eTe(m)*v_light);  % offset in seconds
end
offset = rec_clk_offset*1.e3; % offset in [ms]

figure(2);
plot(offset,'linewidth', 2)  % in [ms]
title({'Receiver Clock Off-set as Determined'; 'by Gaussian Elimination'},'Fontsize',16)
xlabel('Epochs [1 s interval]','Fontsize',16)
ylabel('Clock Off-set [ms]','Fontsize',16)
set(gca,'Fontsize',16);

print -dpdf easy72

%%%%%%%%% end easy7.m %%%%%%%%%