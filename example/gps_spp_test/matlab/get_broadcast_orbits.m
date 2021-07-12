function obs = get_broadcast_orbits(obs,ephemeris,rec_pos)

time = obs.data(:,obs.col.TOW); % time vector for computing position
PRN  = obs.data(:,obs.col.PRN); % PRN of satellite

if isfield(obs.col, 'E') == 0
    obs_size = size(obs.data,2);
    obs.col.E  = obs_size+1;
    obs.col.a  = obs_size+2;
    obs.col.e  = obs_size+3;
    obs.col.XS = obs_size+4;
    obs.col.YS = obs_size+5;
    obs.col.ZS = obs_size+6;
    obs.col.satClkCorr = obs_size+7;
    obs.col.VXS = obs_size+8;
    obs.col.VYS = obs_size+9;
    obs.col.VZS = obs_size+10;
    obs.col.Rel = obs_size+11;
    obs.col.RANGE=obs_size+12;
    obs.col.tsat_b = obs_size+13;
end

% initialize constants and variables
svid = ephemeris(:,1);
m0   = ephemeris(:,2);
dn   = ephemeris(:,3);
e    = ephemeris(:,4);
a    = (ephemeris(:,5)).^2;
omg0 = ephemeris(:,6);
i0   = ephemeris(:,7);
w    = ephemeris(:,8);
odot = ephemeris(:,9);
idot = ephemeris(:,10);
cuc  = ephemeris(:,11);
cus  = ephemeris(:,12);
crc  = ephemeris(:,13);
crs  = ephemeris(:,14);
cic  = ephemeris(:,15);
cis  = ephemeris(:,16);
toe  = ephemeris(:,17);
iode = ephemeris(:,18);
GPS_week = ephemeris(:,19);
toc=ephemeris(:,20);
af0= ephemeris(:,21);
af1= ephemeris(:,22);
af2= ephemeris(:,23);
TGD=ephemeris(:,24);

meu = 3.986005e14;         % earth's universal gravitational [m^3/s^2]
odote = 7.2921151467e-5;   % earth's rotation rate (rad/sec)
lightspeed = 2.99792458e8; % speed of light (m/s)

F = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]

indx1 = find(svid == PRN(1));

% compute positions for single time
tsat = time(1);
for j = 1:4
    [blah indx2] = min(abs(tsat-toe(indx1)));
    if tsat-toe(indx1(indx2)) < 0 
        if indx2 == 1
        else
            indx2 = indx2 - 1;
        end
    end
    index = indx1(indx2);
    obs.data(1,obs.col.a) = a(index);
    obs.data(1,obs.col.e) = e(index);
    n0 = sqrt(meu/a(index)^3);
    t = tsat-toe(index);
    n = n0 + dn(index);
    m = m0(index) + n*t;
    
    m_dot=n;                                  % Calculate Velocity
    
    E = kepOrb2E(m,e(index));
    
    %Compute relativistic correction term
    dtr = F * e(index) * sqrt(a(index)) * sin(E);
    obs.data(1,obs.col.Rel)=dtr*299792458;
    
    % Compute satellite clock correction    
    clkCorr= (af2(index) * (tsat-toc(index)) + af1(index)) * (tsat-toc(index)) + ...
        af0(index);
    obs.data(1,obs.col.satClkCorr) = clkCorr*299792458;
    
    t = t - clkCorr;
    
    E_dot=m_dot/(1-e(index)*cos(E));          % Calculate Velocity    
    
    obs.data(1,obs.col.E) = E;
    v = atan2(sqrt(1-e(index)^2)*sin(E),cos(E)-e(index));
     
    v_dot=sin(E)*E_dot*...                    % Calculate Velocity
        (1+e(index)*cos(v))/(sin(v)*(1-e(index)*cos(E)));  
    
    phi = v + w(index);
    
    phi_dot=v_dot;                            % Calculate Velocity
    
    du = cus(index)*sin(2*phi) + cuc(index)*cos(2*phi);
    dr = crs(index)*sin(2*phi) + crc(index)*cos(2*phi);
    di = cis(index)*sin(2*phi) + cic(index)*cos(2*phi);
    
    du_dot=2*(cus(index)*cos(2*phi)-cuc(index)*sin(2*phi))*phi_dot; % Calculate Velocity
    dr_dot=2*(crs(index)*cos(2*phi)-crc(index)*sin(2*phi))*phi_dot; % Calculate Velocity
    di_dot=2*(cis(index)*cos(2*phi)-cic(index)*sin(2*phi))*phi_dot; % Calculate Velocity
        
    u = phi + du;
    r = a(index)*(1-e(index)*cos(E)) + dr;
    i = i0(index) + di + idot(index)*t;
    
    u_dot=phi_dot+du_dot;                         % Calculate Velocity
    r_dot=a(index)*e(index)*sin(E)*E_dot+dr_dot;  % Calculate Velocity
    i_dot=idot(index)+di_dot;                     % Calculate Velocity

    xp = r*cos(u);
    yp = r*sin(u);
    
    xp_dot=r_dot*cos(u)-r*sin(u)*u_dot;           % Calculate Velocity
    yp_dot=r_dot*sin(u)+r*cos(u)*u_dot;           % Calculate Velocity

    omg = omg0(index) + (odot(index) - odote)*t - odote*toe(index);
    
    omg_dot=odot(index) - odote;                  % Calculate Velocity
    
    obs.data(1,obs.col.XS) = xp*cos(omg) - yp*cos(i)*sin(omg);
    obs.data(1,obs.col.YS) = xp*sin(omg) + yp*cos(i)*cos(omg);
    obs.data(1,obs.col.ZS) = yp*sin(i);
    
    obs.data(1,obs.col.VXS)= xp_dot*cos(omg)-yp_dot*cos(i)*sin(omg)...  % Calculate Velocity
        +yp*sin(i)*sin(omg)*i_dot-obs.data(1,obs.col.YS)*omg_dot;
    obs.data(1,obs.col.VYS)= xp_dot*sin(omg)+yp_dot*cos(i)*cos(omg)...  % Calculate Velocity
        -yp*sin(i)*i_dot*cos(omg)+obs.data(1,obs.col.XS)*omg_dot;
    obs.data(1,obs.col.VZS)= yp_dot*sin(i)+yp*cos(i)*i_dot;             % Calculate Velocity
    
    % compute the range
    R = obs.data(1,obs.col.XS:obs.col.ZS) - rec_pos;
    R = sqrt(sum(R.^2));
    obs.data(1,obs.col.RANGE) = R;    % range
    tau=R/lightspeed;
    
    % Add earth rotation correction here
    phi=-odote*tau;
    corr=[cos(phi),-sin(phi);sin(phi),cos(phi)]*[obs.data(1,obs.col.XS);obs.data(1,obs.col.YS)];
    obs.data(1,obs.col.XS)=corr(1);
    obs.data(1,obs.col.YS)=corr(2);
    % update the range
    R_new = obs.data(1,obs.col.XS:obs.col.ZS) - rec_pos;
    R_new = sqrt(sum(R_new.^2));
    obs.data(1,obs.col.RANGE) = R_new;    % range
    tau_new=R_new/lightspeed;
    tsat = time(1) - tau_new;
end

obs.data(1,obs.col.tsat_b)=tsat;

%still need to include earth rotation here

% ------------------------------------------------
function E = kepOrb2E(M,e)
% Inputs:  - mean anomaly in radians
%          - eccentricity
% Output: Eccentric anomaly

if (-pi < M < 0) | (M > pi)
    E = M - e;
else
    E = M + e;
end

check = 1;

while check > 10e-10
    E_new = (E + (M - E + e * sin(E))/(1 - e * cos(E)));
    check = abs(E_new - E);
    E = E_new;
end

