
% EASY5  computes vector components of a baseline. With given C/A code
%            and carrier phase observations we estimate the ambiguities using integer
%	         least-squares (ILS) or Goad algorithms and estimate the baseline
%            components by a least-squares algorithm. The code does not handle
%	        outliers.


% RINEX version3.03

% Kai Borre 27-07-2002
% Copyright (c) by Kai Borre
% $Revision: 1.1 $  $Date: 2005/01/20  $
% Total revision 2.0, February 17, 2016

% The following code is general, but with the following limitations

% 1. We assume the observational columns are identical and ordered in the
% same sequence in master and rover files. The present observations just
% by chance contain identical satellites which are ordered in the same
% sequence. So sats1 and sats2 are the same. Alternatively, we need to
% establish a matching between sats1 and sats2 and likewise for Obs1
% and Obs2

% 2. There are no satellites with elevation angle less than 10 degrees.
% Minimum is 12 degrees. There are no rising or setting satellites during
% the observation period

% 3. The satellite having highest elevation angle at the start becomes the
% second highest about the middle of the observation file.

% 4. We do not apply tropospheric correction as it is very small in case
% master and rover stations only are about 1 meter apart

% 5. The actual baseline lenght was measured as 0.645 meter. The final
% value for norm(x) = 0.684 m.

% Initial setting and computation of constants
v_light = 299792458;	     % vacuum speed of light m/s
f1 = 154*10.23E6;		     % L1 frequency Hz
f2 = 120*10.23E6;			 % L2 frequency Hz
lambda1 = v_light/f1;	     % wavelength on L1:  .19029367  m
lambda2 = v_light/f2;	     % wavelength on L2:  .244210213 m

% Read RINEX ephemerides file and convert to internal Matlab format
rinexe('log_24h.15n','eph.dat');
Eph = get_eph('eph.dat');

% Selection of observation type
%  ss = 'C1W L1W C2W L2W' %; % pseudorange and carrier phase
ss = 'C1C L1C C2X L2X'; % just to test other obsrervation types
fprintf('\n %s', ss)

% Open the master observation file
ofile1 = 'log_24h.15o';
fid1 = fopen(ofile1,'rt');

% Open the rover observation file
ofile2 = 'log_r.15o';
fid2 = fopen(ofile2,'rt');

ant_delta = [];
linjer = 0;
ij = [];
x = zeros(3,1);

% Gobbling the master file header, and collecting useful information
while 1
    linjer = linjer +1;
    line = fgetl(fid1);
    
    answer = strfind(line,'END OF HEADER');
    if  ~isempty(answer), break; end;
    
    answer = strfind(line,'REC #');
    if ~isempty(answer)
        var = textscan(fid1,'%s %d14','Delimiter','\n');
        var1 = var{1}{1};
        cox = str2double(var1(1:14));
        coy = str2double(var1(15:28));
        coz = str2double(var1(29:42));
        position = [cox; coy; coz];
    end
    
    answer = strfind(line,'ANT #');
    if ~isempty(answer)
        var = textscan(fid1,'%s %d14','delimiter','\n');
        var1 = var{1}{1};
        ax = str2double(var1(1:12));
        ay = str2double(var1(14:25));
        az = str2double(var1(27:38));
        ant_delta = [ax; ay; az];
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
        % the cell array of strings tline1 includes two strings in the start
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
        numSV = str2double(NumberSV);
    end
end % end reading master header

% the string arrays for the tline1 and tline2 contain an integer after the
% carrier phase observation. We must account for this by the following
% correctional table.

% the following code may be done more elegantly
for qq = 1:4
    if qq == 1, a0 = 6; b0 = 7; end
    if qq == 2, a0 = 6; b0 = 7; end
    if qq == 3, a0 = 14; b0 =15; end
    if qq == 4, a0 =14; b0 =15; end
    if strcmp(ss(a0:b0), '1C'), obs_col(1,qq) = obs_col(1,qq)+1;
    elseif  strcmp(ss(a0:b0), '1W'), obs_col(1,qq) = obs_col(1,qq)+2;
    elseif strcmp(ss(a0:b0),  '2X'), obs_col(1,qq) = obs_col(1,qq) +3;
    elseif strcmp(ss(a0:b0),  '2W'), obs_col(1,qq) = obs_col(1,qq) +4;
    else  strcmp(ss(a0:b0),  '5X'), obs_col(1,qq) = obs_col(1,qq) +5;
    end;
end;

Omc = [];
base = [];
epoch =  0;

time = textscan(fid1,'%s %d8','Delimiter','\n');
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
    obs1 = textscan(fid1,'%s %d8','Delimiter','\n');
    obs1x = obs1{1}{1} ;
    obs1y = strsplit(obs1x);
    sat1 = obs1y{1} ;
    sats1(i,:) = str2double(sat1(2:3));
    Obs1(i,:) = str2double(obs1y(obs_col));
end

% Gobbling the rover file header, and collecting useful information
% the whole story repated for the rover with index 2
% we start by gobbeling the header
for qq = 1:linjer +3 %1
    aa = fgetl(fid2);
end

% Reading header of observation record in rover file
time = textscan(fid2,'%s %d8','Delimiter','\n');
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

Obs2 = zeros(NoSvs2,length(obs_col));
for i = 1:NoSvs2
    obs2 = textscan(fid2,'%s %d8','Delimiter','\n');
    obs2y = obs2{1}{1};
    obs2 = strsplit(obs2y);
    sat2 = obs2{1};
    sats2(i,:) = str2double(sat2(2:3));
    Obs2(i,:) = str2double(obs2(obs_col));
end

% Establish synchronous reading of fid1 and fid2
% The epoclear ch interval is int seconds
if  time1 > time2
       for tq = 1: round(time1-time2) 
        time = textscan(fid2,'%s %d8','Delimiter','\n');
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
        
        Obs2 = zeros(NoSvs2,length(obs_col));
        for i = 1:NoSvs2
            obs2 = textscan(fid2,'%s %d8','Delimiter','\n');
            obs2y = obs2{1}{1};
            obs2 = strsplit(obs2y);
            sat2 = obs2{1};
            sats2(i,:) = str2double(sat2(2:3));
            Obs2(i,:) = str2double(obs2(obs_col));
        end
    end
end

if  time1 < time2
    % In the present data time2-time1 is 1 second. This is,
    % we need to read one more record at the master site
    % before we are in sync.
    time = textscan(fid1,'%s %d8','Delimiter','\n');
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
        obs1 = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1};
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1};
        sats1(i) = str2double(sat1(2:3));
        Obs1(i,:) = str2double(obs1y(obs_col));
    end
end % synchronization

fprintf('\nFirst Common Epoch Time1 :  %8.0f', time1)
fprintf('\nFirst Common Epoch Time2 :  %8.0f\n', time2)

disp(' ')
disp(' ... Now wait  a few minutes for further results and a plot!')
X_jacc = [];
x_acc = [];

while 1
    epoch = epoch +1;
    % Reading header of observation block at Master
    time = textscan(fid1,'%s %d8','Delimiter','\n');
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
    
    % Reading observation block at Master
    Obs1 = zeros(NoSvs1,length(obs_col));
    for i = 1:NoSvs1
        obs1 = textscan(fid1,'%s %d8','Delimiter','\n');
        obs1x = obs1{1}{1} ;
        obs1y = strsplit(obs1x);
        sat1 = obs1y{1} ;
        sats1(i) = str2num(sat1(2:3));
        Obs1(i,:) = str2double(obs1y(obs_col));
    end
    
    % Reading header of observation record in rover file
    time = textscan(fid2,'%s %d8','Delimiter','\n');
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
    
    Obs2 = zeros(NoSvs2,length(obs_col));
    for i = 1:NoSvs2
        obs2 = textscan(fid2,'%s %d8','Delimiter','\n');
        obs2y = obs2{1}{1};
        obs2 = strsplit(obs2y);
        sat2 = obs2{1};
        sats2(i,:) = str2double(sat2(2:3));
        Obs2(i,:) = str2double(obs2(obs_col));
    end
    
    % Next we test if all observed SVs have an ephemeris.
    % sats1 contains the SVs as read in the observation lines
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the follwing manipulations
    Sats = intersect(sats1,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats1
    lia = ismember(sats1,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats1(lia==0) = [];
    Obs1(lia==0,:) = []; % modfied because of two columns
    NoSV1 = length(sats1);
    
    % Next we test if all observed sats have an ephemeris.
    % sats contains the SVs as read in the observation lines.
    % The intersect command delivers Sats in sorted order!
    % Therefore we must be careful in the following manipulations
    Sats = intersect(sats2,Eph(1,:));
    
    % The command ismember does not change the sequence of entries in sats
    lia = ismember(sats2,Sats);
    % A 0 (zero) in lia indicates that a SV has been deleted. We delete the
    % corresponding row in the observations
    sats2(lia==0) = [];
    Obs2(lia==0, :) = [];
    NoSV2 = length(sats2);
    % end reading first record in rover file
    
    % By chance sats1 and sats 2 are the same in the sample data. Otherwise
    % we need to establish  a matching between sats1 and sats2 and
    % likewise for Obs1 and Obs2
    
    % We compute elevation angles for all sats1 and sats2
    [X_i, el] = recpo_ls(Obs1(:,3), sats1, time1, Eph);
    [X_j0,~] = recpo_ls(Obs2(:,3), sats2, time1, Eph);
    if epoch == 1
        X_j = X_j0(1:3,1);
    end
    % if epoch == 1
    % forcing the choice of reference satellite to be the second highest
    % at the start
    %  sats1
    % refSV = 5
    %  sats1(refSV) = [];
    % end
    
    if epoch == 1
        % The SV with largest elevation is taken as reference SV
        [y,refSV] = max(el) ;
        old_refSV = refSV;
        fprintf('\n Reference satellite is chosen as  PRN %3.0f\n', sats1(refSV))
        if refSV ~= old_refSV
            fprintf('\n New reference satellite is chosen as  PRN  %3.0f\n\n', sats1(refSV))
            sats1 = union(sats1, old_refSV);
            % no  PRN 12 must be inserted at the entry from where it was deleted!!!
        end
    end
    %The refSV is PRN 12 has index 6 in sats1
    % Finding columns in Eph for each SV
    m = NoSV1;
    col_Eph = zeros(1,m);
    for t = 1:m
        col_Eph(t) = find_eph(Eph,sats1(t),time1+dt1);
    end
    
    m1 = m-1; % m1 is all sv.s minus refSV
    X = zeros(3+2*m1,1);
    
    % creating a row of indices for all Svs minus the refSV
    t = 1: NoSV1;
    t1 = setdiff(t,refSV);
    % Computation of covariance matrix Sigma for double differenced observations
    D = [ones(m1,1) -eye(m1) -ones(m1,1) eye(m1)];
    Sigma = D*D';
    
    % A block for estimation of AMBIGUITIES
    % we accumulate the normals for the first 10 epochs
    switch(epoch)
        
        case {1, 2, 3, 4, 5, 6, 7, 8, 9,10}
            
            N = zeros(3+2*m1,3+2*m1);	  % initialization of normals
            rs = zeros(3+2*m1,1);	            % initialization of right side
            
            % Computing rho for refSV
            [~,rhok_j,~] = get_rho(time1, Obs2(refSV,1), Eph(:,col_Eph(refSV)), X_j);
            [~,rhok_i,Xk_ECF] = get_rho(time1, Obs1(refSV,1), Eph(:,col_Eph(refSV)), X_i);
            
            tt = 0;
            ts = length(t1);
            b = zeros(ts,1);
            bk = zeros(ts,1);
            A1 = [];
            for t = t1
                tt = tt+1;
                [~,rhol_j, ~] = get_rho(time1,Obs2(t,1), Eph(:,col_Eph(t)), X_j);
                [~,rhol_i, Xl_ECF] = get_rho(time1,Obs1(t,1), Eph(:,col_Eph(t)), X_i);
                A0 = [(Xk_ECF(1)-X_j(1))/rhok_j - (Xl_ECF(1)-X_j(1))/rhol_j  ...
                    (Xk_ECF(2)-X_j(2))/rhok_j - (Xl_ECF(2)-X_j(2))/rhol_j ...
                    (Xk_ECF(3)-X_j(3))/rhok_j - (Xl_ECF(3)-X_j(3))/rhol_j];
                A1 = [A1; A0];
                Phi1 = (Obs1(refSV,2)-Obs1(t,2)-Obs2(refSV,2)+Obs2(t,2))*lambda1;
                Phi2 = (Obs1(refSV,4)-Obs1(t,4)-Obs2(refSV,4)+Obs2(t,4))*lambda2;
                b(tt,:) =                   Phi1-lambda1*X(3+tt,1);
                b(length(t1)+tt,:) = Phi2-lambda2*X(3+length(t1)+tt,1);
                bk(tt,:) =                    rhok_i-rhok_j-rhol_i+rhol_j;
                bk(length(t1)+tt,:) =  rhok_i-rhok_j-rhol_i+rhol_j;
            end;
            A_modi = eye(m1);	    	          % modified coefficient matrix
            A_modi(:,refSV) = -ones(m1,1);
            A_aug = [A1 lambda1*A_modi 0*eye(m1); A1 0*eye(m1) lambda2*A_modi];
            N = N+A_aug'*kron(eye(2),Sigma)*A_aug;
            rs = rs+A_aug'*kron(eye(2),Sigma)*(b-bk);
            
            
        case 11
            
            PP = pinv(N);
            % X contains the three baseline components and next the float ambiguities
            X = PP*rs;
            % The next step is to convert the real ambiguities in X (all entries below the
            % fourth component and to the end) into integers. We indicate two algorithms:
            
            % the Integer Least-Squares (ILS) algorithm
            PP = 0.5*(PP+PP');  %to make PP symmetric
            [a, sqnorm] =  ILS( X(4:4+2*m1-1,1), PP(4:4+2*m1-1,4:4+2*m1-1),1);
            % Correcting to baseline vector as consequence of changing float ambiguities
            % to fixed ones
            X(1:3,1) = X(1:3,1)-PP(1:3,4:4+2*m1-1)*inv(PP(4:4+2*m1-1,4:4+2*m1-1))*...
                (X(4:4+2*m1-1,1)-a(:,1)); %select SECOND  first set of candidates
            % X(4:4+2*m1-1,1) = a(:,1);
            %a = a(:,2); % forces the second set of N values in use
            
            % the Goad algorithm
            %for j = 1:tt
            %    K1 = round(X(3+tt)-X(3+m1+tt));
            %   K2 = round(60*X(3+tt)-77*X(3+m1+tt));
            %   trueN2 = round((60*K1-K2)/17);   % (15.10)
            %   trueN1 = round(trueN2+K1);         % (15.11)
            %    amb(j,1:2) = [trueN1 trueN2];
            % end
            %a =  [amb(:,1);  amb(:,2) ];
            
            fprintf('\n N1 for PRN %3.0f: %3.0f', [sats1(t1)'; a(1:m1,1)'])
            fprintf('\n')
            fprintf('\n N2 for PRN %3.0f: %3.0f',[sats1(t1)'; a(m1+1:2*m1,1)'])
            fprintf('\n')
            
            % Setting covariances for the Kalman filter; the state vector contains (x,y,z)
            P =  0.01*eye(3);	                             		 % covariances of state vector
            Q = 0.05^2*eye(3);			                          % covariances of system
            R = 0.005^2*kron(eye(2),inv(Sigma));	    % covariances of observations
            [phi_j,lambda_j,h_j] = togeod(6378137, 298.257223563, X_j(1), X_j(2), X_j(3));
            h_i = h_j;
            
        otherwise
            
            % estimate of baseline components
            tt = 0;  % a counter
            A = zeros(m1,3); %zeros(2*m1,3);
            % Computing rho for refsv
            [torr2,rhok_j,~] = get_rho(time2, Obs2(refSV,3), Eph(:,col_Eph(refSV)), X_j);
            [torr1,rhok_i,Xk_ECF] = get_rho(time1, Obs1(refSV,3), Eph(:,col_Eph(refSV)), X_i);
            for t = t1
                tt = tt+1;
                [torr4,rhol_j, ~] = get_rho(time2,Obs2(t,3), Eph(:,col_Eph(t)), X_j);
                [torr3,rhol_i,Xl_ECF] = get_rho(time1,Obs1(t,3), Eph(:,col_Eph(t)), X_i);
                A0 = [(Xk_ECF(1)-X_j(1))/rhok_j - (Xl_ECF(1)-X_j(1))/rhol_j  ...
                    (Xk_ECF(2)-X_j(2))/rhok_j - (Xl_ECF(2)-X_j(2))/rhol_j ...
                    (Xk_ECF(3)-X_j(3))/rhok_j - (Xl_ECF(3)-X_j(3))/rhol_j];
                A(tt,:) = A0;
                A(m1+tt,:) = A0;
                %time_corr = (torr1-torr2-torr3-torr4);
                % Composing the right side
                
                % Tropospheric correction using standard meteorological parameters
                %[ ~,el_ki, ~] = topocent(X_i(1:3),Xk_ECF-X_i(1:3));
                %[ ~,el_li,~] = topocent(X_i(1:3),Xl_ECF-X_i(1:3));
                %[ ~,el_kj, ~] = topocent(X_j(1:3),Xk_ECF-X_j(1:3));
                %[az,el_lj,d] = topocent(X_j(1:3),Xl_ECF-X_j(1:3));
                %%el_ki,    el_li,    el_kj,    el_lj
                %t_corr = tropo(sin(el_lj*pi/180),...
                %    h_j*1.e-3,1013,293,50,0,0,0)...
                %    -tropo(sin(el_li*pi/180),....
                %    h_i*1.e-3,1013,293,50,0,0,0)...
                %   -tropo(sin(el_kj*pi/180),...
                %   h_j*1.e-3,1013,293,50,0,0,0)...
                %   +tropo(sin(el_ki*pi/180),...
                %   h_i*1.e-3,1013,293,50,0,0,0);
                t_corr = 0;
                b(tt,:) = (Obs1(refSV,2)-Obs1(t,2)-Obs2(refSV,2)+Obs2(t,2)  ...
                    -a(tt,1))*lambda1-t_corr;
                b(m1+tt,:)  = (Obs1(refSV,4)-Obs1(t,4)-Obs2(refSV,4)+Obs2(t,4) ...
                    -a(m1+tt,1))*lambda2-t_corr;
                bk(tt,:) =         ( rhok_i-rhok_j-rhol_i+rhol_j);
                bk(m1+tt,:) =  (rhok_i-rhok_j-rhol_i+rhol_j);
            end; % t
            N = [A1;A1]'*[Sigma zeros(m1,m1);zeros(m1,m1) Sigma]*[A1;A1];
            rs = [A1;A1]'*[Sigma zeros(m1,m1); zeros(m1,m1) Sigma]*(b-bk);
            x = inv(N)*rs;
            X_j = X_j+x;
            base = [base X_j-X_i(1:3)];
            X_jacc = [X_jacc X_j];
            x_acc = [x_acc x];
    end % switch
    X_j = X_i(1:3,:)+x;
    if feof(fid1) == 1, break, end;
    if feof(fid2) == 1, break; end;
end % while

X = X_j-X_i(1:3,1);
%e = zeros(1,epoch-11);
%n = e;
%u = e;
% Transformation of geocentric baseline coordinates into topocentric coordinates
for i = 1:epoch-11
    [e(i),n(i),u(i)] = xyz2enu(phi_j,lambda_j,base(1,i),base(2,i),base(3,i));
end
fprintf('\n\nBaseline Components\n')
fprintf('\nX: %8.3f m,  Y: %8.3f m,  Z: %8.3f m\n',X(1),X(2),X(3))
fprintf('\nE: %8.3f m,  N: %8.3f m,  U: %8.3f m\n',mean(e),mean(n),mean(u))

figure(1);
plot(1:epoch-11, x_acc*1000)
title('Baseline','fontsize',14)
ylabel('({\itX}, {\itY}, {\itZ}) of baseline [mm]','fontsize',14)
xlabel('Epochs [1 s interval]','fontsize',14)
legend('\itX','\itY','\itZ')
set(gca,'fontsize',14)
legend

print -dpdf easy51

figure(2);
plot(1:epoch-11,[(e-e(1))' (n-n(1))' (u-u(1))']*1000,'linewidth',.2)
title('Variation in baseline','fontsize',14)
ylabel('Chance relative to initial position [mm]','fontsize',14)
xlabel('Epochs [1 s interval]','fontsize',14)
legend('East','North','Up')
set(gca,'fontsize',14)
legend

print -dpdf easy52
%%%%%%%%%%%%%%%%%%%%%% end easy5.m  %%%%%%%%%%%%%%%%%%%