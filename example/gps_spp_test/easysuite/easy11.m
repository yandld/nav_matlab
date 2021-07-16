% easy11    Creates a stereographic plot of GPS satellite  orbits
%                from an epehemerides or almanac file. The plot is as
%                seen from the position (phi, lambda). All (parts of)
%                orbits with elevation angle lower than a given
%                cut-off angle (mask) are omitted.
%                As input we use the ephemerides in our basic RINEX
%               navigation file. It only contains nine ephemerides. The
%               result shows two windows with a considerable amount
%               of visible satellites. This is like the actual situation in the
%               early 1990es. Then you could have a window  of about
%               one hour at say 5--6 am and this repeated 12 hours later.
%
%              An additional plot is created showing number
%              of visible satellites and when they are visible.
%              Finally a stem plot depicts the number of
%             satelliets visible for how long
%
%             Alternatively, if you want to include all GPS satellites,
%             we need an almanac file which can be downloaded from
%             http://www.navcen.uscg.gov/?pageName=gpsAlmanacs

%Kai Borre 26-08-2008
%Copyright (c) by Kai Borre
%$Revision: 1.1 $  $Date: 2009/04/05  $
% Total revision January 10, 2016

% RINEX version 3.03

set(0,'DefaultTextFontName','Times');
set(0,'DefaultAxesFontName','Times');
set(0,'DefaultTextFontSize',12);

% If you want to use a RINEX ephemeris file uncomment
% line 37 and comment line 41 out.

% the following five assignments are mandatory
%%rinexe('log_24h.15n','eph.dat');
% in  case you want to include all GPS satellites in the constellation,
% we need an almanac. This is read and converted to the usual
% eph format by the following call
rinexa('current.alm','eph.dat');

ephemerides = 'eph.dat';
mask = 10;
phi = [57 12 43];
lambda = [50 10 39];

%reading ephemerides
fide = fopen(ephemerides,'r');
Eph = fread(fide,inf,'double');
m = length(Eph);
eph = reshape(Eph,21,m/21);

% transformation of given location (phi,lambda,h) to (X,Y,Z)
Phi = dms2rad(phi(1),phi(2),phi(3));
Phi = Phi*180/pi;
Lambda = dms2rad(lambda(1),lambda(2),lambda(3));
Lambda = Lambda*180/pi;
[M(1,1),M(2,1),M(3,1)] = frgeod(6378137,298.257222101,Phi,Lambda,0);

% Computation of (azimuth, elevation) for each satellite
% for each 15 min.s. We use only one ephemeris for each PRN.
% Anyway, the plot is only for visual use
[prns, ind] = unique(eph(1,:));
satrows = length(ind);
 az = ones(satrows,96)*inf; % satrows is number of PRN and 96 quaters of an hour in 24 hours
el = ones(satrows,96)*inf;

start_time = max(eph(21,:));
for sat = ind'
    j = 0;
    for time = start_time:900:start_time+86400
        S = satpos(time,eph(:,sat)); %sat
        j = j+1;
        [azimuth,elevation,distance] = topocent(M,S-M);
        az(sat,j) = azimuth; % sat runs over PRN, j runs over sow
        el(sat,j) = elevation;
    end
end
% We assume that there are satrows PRNs
XX = zeros(satrows,40)*inf; % a matrix of NaNs to store plot data
YY = XX;

figure(1);
% polarhg draws coordinate lines of a polar plot. We add
% circles with radii 30 and 60 degrees
polarhg([30 60])
hold on
for k = ind' % 1:32
    % if az(k,1) == 0, break, end  % continue
    AZ = az(k,:);
    EL = el(k,:);
    % remove data below the cut-off angle
    AZ(find(EL <= mask)) = nan;
    EL(find(EL <= mask)) = nan;
    % conversion from polar to rectangular coordinates
    xx = (90-EL).*cos(AZ*pi/180);
    yy = (90-EL).*sin(AZ*pi/180);
    XX(k,1:length(xx)) = xx;
    YY(k,1:length(yy)) = yy;
end % k
%the first coord. moves text vertically (increasing values up),
% the second coord. moves text horizontally (increasing values right)
text(135,-95,{['Skyplot for the position (\phi, \lambda) = (' ...
    num2str(round(Phi)) '\circ, '  num2str(round(Lambda)) '\circ)']})
text(115,-45,{['Elevation mask  ' num2str(mask) '\circ' ]}) %120
text(-120,-120,['All PRNs except  ' num2str(setdiff(1:32,prns)) ])
plot(XX',YY','linewidth',2)
hold off
set(gca,'Fontsize',9);

print('easy111','-dpdf')


% preparation for visibility plot  %%%

% we choose a resolution of 5 min.s,
% ie. 24 hours times 12 = 288 which becomes the range of j
satsum = zeros(1,288);
visible = zeros(2*(size(prns,2)+1),288);

for sat = ind'
    %Az = [];
    %El = [];
    i = eph(1,sat);
    for j = 1:288
        time = 300*(j-1); % in s, 0<time<86400
        S = satpos(time,eph(:,sat));
        [az,el,d] = topocent(M,S-M);
        if el > mask
 %           Az = [Az az];
  %          El = [El el];
            satsum(j) = satsum(j)+1;
            visible(2*i,j) = 1;
        end
    end
end

figure(2);
set(gca,'Fontsize',16);
area(satsum)
set(gca,'XTick',1:71:288)
set(gca,'XTickLabel',{'0','6','12','18','24'})
xlabel('GPS Time [hours]')
ylabel('# of Visible Satellites')
title(['Elevation Mask ' num2str(mask) '\circ'])
colormap summer

print -dpdf easy112

figure(3);
set(gca,'Fontsize',16);
imagesc(flipud(visible));
colormap(autumn(5))
set(gca,'XTick',1:71:288)
set(gca,'XTickLabel',{'0','6','12','18','24'})
set(gca,'YTick',-3:16:(2*(size(prns,2)+1)))
set(gca,'YTickLabel',{'2','8','16','24','32'});
xlabel('GPS Time [hours]')
ylabel('PRNs')
title('Yellow Lines Indicate Visible Satellites')
colormap winter

print -dpdf easy113

figure(4);
set(gca,'Fontsize',16);
an = sum(visible,1);
% we assume the total # of PRNs always is between 1 and 20
total = zeros(1,20);
for i = 1:288
    j = an(i);
    if j == 0, j = j+1; total(j) = 0.01; continue, end
    total(j) = total(j)+1;
end
% the adder total adds per 5 minutes, division by 12 changes to per hour
bar(1:20,total/12);  % 10-> 32
title('Number of Visible Satellites')
ylabel('Hours')

print -dpdf easy114

%%%%%%%%%%%%%%%%%%%%% end easy11.m %%%%%%%%%%%%%%
