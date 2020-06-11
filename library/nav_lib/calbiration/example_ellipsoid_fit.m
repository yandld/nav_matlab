
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% simulate data 
c = [-0.5; 0.2; 0.1]; % ellipsoid center
r = [1; 1; 2]; % semiaxis radii

[x,y,z] = ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3),6);
D = [x(:),y(:),z(:)];

% add noise:
n = length(D);
noiseIntensity = 0.01; %噪声强度 
D = D + randn(n,3) * noiseIntensity;

%%%real data

mag= [
-16.2500 -31.5833 -59.0000 
-21.6000 -51.2917 -37.6250 
-26.5583 -56.4583 -11.4583 
  5.0833 -15.0000 -64.3500 
  7.8333  11.6833 -65.2083 
 -9.4333  32.6833 -62.3917 
-42.3750  -8.2250 -61.4333 
-40.8083  21.3083 -60.1250 
 27.5000  28.5417 -52.4750 
 36.4333   2.4750 -50.6667 
 32.5417 -21.8750 -46.1417 
 48.9333  16.4333 -35.5833 
    ];

D = mag./1.0;
% D = D(1:15,:);
n = length(D);

%% remove outliter
% 计算各个点到平均点的欧氏距离
% m = mean(D);
% for i=1:length(D)
%     d(i) = norm(D(i,:) - m);
% end
% m = mean(d);
% for i=1:length(d)
%     if(d(i) > 3*m)
%         D(i,:) =[];
%     end
% end

%% matlab internal fitting 

[A, b, expmfs] = magcal(D, 'auto')
%fprintf( 'away from cetner %.5g\n', norm( b' - c) );
C = (D-b)*A; % calibrated data

figure(1)
plot3(D(:,1),D(:,2),D(:,3),'LineStyle','none','Marker','X','MarkerSize',2)
hold on
grid(gca,'on')
plot3(C(:,1),C(:,2),C(:,3),'LineStyle','none','Marker', ...
            'o','MarkerSize',2,'MarkerFaceColor','r') 
axis equal
xlabel('uT'); ylabel('uT');zlabel('uT') 
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off


figure;
plot(D(:,1), D(:,3) ,'LineStyle','none','Marker','X','MarkerSize',2);
hold on;
grid(gca,'on')
plot(C(:,1), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off

figure;
plot(D(:,2), D(:,3) ,'LineStyle','none','Marker','X','MarkerSize',2);
hold on;
grid(gca,'on')
plot(C(:,2), C(:,3),'LineStyle','none','Marker',   'o','MarkerSize',2,'MarkerFaceColor','r');
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
axis equal
hold off




% %%  elliposid test 1 
% [A, b, expmfs, er]=ellipsoid_fit(D, 5);
% 
% A
% b
% expmfs
% er



%% DIP13 test
% feature_scale = 50;
% mag = mag ./ feature_scale;
% 
%  [mis, bias, lamda, inter, J] = dip13 (acc, mag, 1, 0.001);
% 
%  fprintf('using DIP13 mehold:\n');
%  
%  figure;
%  plot(1:length(J), J, '*-');
%   title('DIP13');
%   
% %fprintf('inter = %d\n', inter);
% %mis = mis ./ mis(1,1);
% mis
% 
% bias' .* feature_scale
% fprintf('incli: %f\r\n',  rad2deg(acos(lamda)) - 90);
