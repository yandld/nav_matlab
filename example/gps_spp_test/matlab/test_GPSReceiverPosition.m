clc
clear
format long g

% read observation file to get orbit elements
[obs,rec_xyz] = read_rinex_obs('madr2000.06o');

% read navigation file to get orbit elements
ephemeris = read_rinex_nav('brdc2000.06n');

epochs = unique(obs.data(:, obs.col.TOW));
TimeSpan=epochs(1:2880);

% Broadcast Orbit
satOrbits.XS=zeros(1,length(TimeSpan));
satOrbits.YS=zeros(1,length(TimeSpan));
satOrbits.ZS=zeros(1,length(TimeSpan));
satOrbits.VXS=zeros(1,length(TimeSpan));
satOrbits.VYS=zeros(1,length(TimeSpan));
satOrbits.VZS=zeros(1,length(TimeSpan));
satOrbits.clk=zeros(1,length(TimeSpan));
satOrbits.Rel=zeros(1,length(TimeSpan));

% GPS Satellite Measurements
c = 2.99792458e8 ; % speed of light (m/s)
fL1 = 1575.42e6;   % L1 frequency (Hz)
fL2 = 1227.6e6;    % L2 frequency (Hz)
B=fL2^2/(fL2^2-fL1^2);
A=-B+1;
satOrbits.C1=zeros(1,length(TimeSpan));
satOrbits.L1=zeros(1,length(TimeSpan));
satOrbits.P2=zeros(1,length(TimeSpan));
satOrbits.L2=zeros(1,length(TimeSpan));
satOrbits.P3=zeros(1,length(TimeSpan)); % Iono free pseudorange
satOrbits.CorrP1=zeros(1,length(TimeSpan)); % Corrected Pseudorange from broadcast orbit
satOrbits.CorrP2=zeros(1,length(TimeSpan)); % Corrected Pseudorange from precise orbit
satOrbits.TOW=TimeSpan';
satOrbits.PRN=0;

satOrbits = repmat(satOrbits,1,32);
for ii=1:32
    satOrbits(ii).PRN=ii;
end

% Initialize User Position
userPos=zeros(length(TimeSpan),4);

for ii=1:length(TimeSpan)    
    this_TOW = TimeSpan(ii);
    index = find(obs.data(:,obs.col.TOW) == this_TOW);
    curr_obs.data = obs.data(index, :);
    curr_obs.col = obs.col;
    
    for jj=1:size(curr_obs.data,1)        
        PRN_obs.data = curr_obs.data(jj,:);
        PRN_obs.col = curr_obs.col;
        
        % Record Measurements
        satOrbits(PRN_obs.data(PRN_obs.col.PRN)).C1(ii)=PRN_obs.data(PRN_obs.col.C1);
        satOrbits(PRN_obs.data(PRN_obs.col.PRN)).L1(ii)=PRN_obs.data(PRN_obs.col.L1);
        satOrbits(PRN_obs.data(PRN_obs.col.PRN)).P2(ii)=PRN_obs.data(PRN_obs.col.P2);
        satOrbits(PRN_obs.data(PRN_obs.col.PRN)).L2(ii)=PRN_obs.data(PRN_obs.col.L2);
        
        % Calculate Iono Free Measurement
        P1 = satOrbits(PRN_obs.data(PRN_obs.col.PRN)).C1(ii);
        P2 = satOrbits(PRN_obs.data(PRN_obs.col.PRN)).P2(ii);
        P3=A*P1+B*P2;
        satOrbits(PRN_obs.data(PRN_obs.col.PRN)).P3(ii)=P3;
    end
    
    stop = 10;
    while stop ~= 1
        for jj=1:size(curr_obs.data,1)
            PRN_obs.data = curr_obs.data(jj,:);
            PRN_obs.col = curr_obs.col;
            
            % Obtain the broadcast orbits
            PRN_obs = get_broadcast_orbits(PRN_obs,ephemeris,rec_xyz');
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).XS(ii)=PRN_obs.data(PRN_obs.col.XS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).YS(ii)=PRN_obs.data(PRN_obs.col.YS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).ZS(ii)=PRN_obs.data(PRN_obs.col.ZS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).VXS(ii)=PRN_obs.data(PRN_obs.col.VXS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).VYS(ii)=PRN_obs.data(PRN_obs.col.VYS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).VZS(ii)=PRN_obs.data(PRN_obs.col.VZS);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).clk(ii)=PRN_obs.data(PRN_obs.col.satClkCorr);
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).Rel(ii)=PRN_obs.data(PRN_obs.col.Rel);            
            
            % Calculate corrected pseudorange based on broadcast orbit
            satOrbits(PRN_obs.data(PRN_obs.col.PRN)).CorrP1(ii)=...
                satOrbits(PRN_obs.data(PRN_obs.col.PRN)).P3(ii)+...
                satOrbits(PRN_obs.data(PRN_obs.col.PRN)).clk(ii)+satOrbits(PRN_obs.data(PRN_obs.col.PRN)).Rel(ii);            
        end
        
        % Calculate User Position
        [broadcast_obs,~]=createObs(this_TOW,satOrbits);
        delta_xyz = comp_pos(broadcast_obs,rec_xyz');
        rec_xyz = rec_xyz + delta_xyz(1:3);
        
        stop=stop-1;
    end
    userPos(ii,1:4) = [rec_xyz; delta_xyz(4)]';
    [lon1(ii),lat1(ii),alt1(ii)] = Geodetic(rec_xyz);
end

fprintf('mean of user position:');
fprintf('%16f%16f%16f\n',mean(userPos(:,1)),mean(userPos(:,2)),mean(userPos(:,3)));
fprintf('delta_xyz:');
fprintf('%16f\n',mean(userPos(:,4)));

