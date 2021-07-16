% EASY17  Drawing GPS constellation in inertial and ECEF frames.
%              The third plot illustrates computation of sub-satellite points
%
%             To create only once the internal format of the
%             almanac we proceed through the following steps:
%                1. Download 'current.alm' from
%                     http://www.navcen.uscg.gov/?pageName=gpsAlmanacs
%                2. Call rinexa('current.alm','eph.dat');
%                3. Run this file easy17

%Kai Borre 16-07-2008
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 2008/07/16  $
% Revision 1.1  made on November 11, 2015

set(0,'DefaultTextFontName','Times');
set(0,'DefaultAxesFontName','Times');
set(0,'DefaultTextFontSize',16);


rinexa('current.alm','eph.dat');
eph = get_eph('eph.dat');
[b,i,j] = unique(eph(1,:),'first'); % i contains the unique columns of eph
n = length(i); % total number of SVs
sv = eph(1,i); % sv contains SV numbers

%%%%%%%%%%%%% inertial version  %%%%%%%%%%%%%%%%%%
figure(1);
hold on
p = zeros(48,3);
for t = 1:48
    p(t,:) = satposin(eph(21,1)+900*t, eph(:,i(1)))';
end
orbit = animatedline('linewidth', .5);

ii = 1;
for k = 2:n
    for t = 1:48
        p(t,:) = satposin(eph(21,1)+900*t, eph(:,i(k)))';
    end
    ii = ii+1; % for change of color
     cc = i(ii);
    set(orbit,'color', [0.15, cc/1000,  cc/200]);
    addpoints(orbit, [p(:,1); nan]', [p(:,2); nan]', [p(:,3); nan]');
    drawnow update;
end

[x,y,z] = sphere(50);
load topo;
props.AmbientStrength = 0.1;
props.DiffuseStrength = 1;
props.SpecularColorReflectance = .3; %.5;
props.SpecularExponent = 20;
props.SpecularStrength = 1;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.Cdata = topo;
surface(6700000*x,6700000*y,6700000*z,props);
light('position',[-1 0 1]); % -1 0 1
light('position',[1.5 -0.5 -0.5], 'color', [.6 .2 .2]); % -1.5 .5 -.5
view([-90 0]) % az = -90 and el = 0
axis equal off
hold off

print -dpdf easy171

%%%%%%%%%%%%%% ECEF version %%%%%%%%%%%%%%%%%%%

figure(2);
hold on
for t = 1:48
    p(t,:) = satpos(eph(21,1)+900*t, eph(:,i(1)))';
end
orbit = animatedline('color',[0.15+.05*i(1) .25+.03*i(1) .09],...
    'linewidth',.5);
for k = 2:n
    for t = 1:48
        p(t,:) = satpos(eph(21,1)+900*t, eph(:,i(k)))';
    end
    addpoints(orbit, [p(:,1); nan]', [p(:,2); nan]', [p(:,3); nan]');
end

[x,y,z] = sphere(50);
load topo;
props.AmbientStrength = 0.1;
props.DiffuseStrength = 1;
props.SpecularColorReflectance = .3; %.5;
props.SpecularExponent = 20;
props.SpecularStrength = 1;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.Cdata = topo;
surface(6700000*x,6700000*y,6700000*z,props);
light('position',[1 0 1]); % -1 0 1
light('position',[1.5 -0.5 -0.5], 'color', [.6 .2 .2]); % -1.5 .5 -.5
view([180 0]) % az and el found from experiments
axis equal off
hold off

print -dpdf easy172

%%%%%%% plot of sub-satellite points for satellite k  %%%%%%%%%%

% The sub-satellite points are computed as the inersection between a sphere
% of radius 6,700 km and the line from origin to the satellite.
% A more refined computation will substitute the sphere with an ellipsoid
% which leads to more evolved computations. Many applications most
% likely only need the present accuracy

figure(3);
hold on
for k = 2 %:n
    acc_proj= zeros(48,3);
    for t = 1:48
        p(t,:) = satpos(eph(21,1)+900*t, eph(:,i(k)))';
        proj = 6700000*p(t,:)/norm(p(t,:)); % t,:
        acc_proj = [acc_proj; proj];
    end
end
[x,y,z] = sphere(50);
load topo;
props.AmbientStrength = 0.1;
props.DiffuseStrength = 1;
props.SpecularColorReflectance = .3; %.5;
props.SpecularExponent = 20;
props.SpecularStrength = 1;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.Cdata = topo;
surface(6700000*x,6700000*y,6700000*z,props);
light('position',[1 0 1]); % -1 0 1
light('position',[1.5 -0.5 -0.5], 'color',[.6 .2 .2]); % -1.5 .5 -.5
view([150 0]) % az and el found from experiments
axis equal off
plot3(acc_proj(:,1),acc_proj(:,2),acc_proj(:,3),'linewidth',2,'color','r')
hold off
set(gcf, 'InvertHardCopy', 'off');

print -dpdf easy173

%%%%%%%%%%%%% end easy17.m %%%%%%%%%%%%%%%%%%%%%%%


