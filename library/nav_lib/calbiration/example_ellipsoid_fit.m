
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% simulate data 
c = [-0.5; 0.2; 0.1]; % ellipsoid center
r = [1; 1; 1]; % semiaxis radii

[x,y,z] = ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3),6);
D = [x(:),y(:),z(:)];

% add noise:
n = length(D);
noiseIntensity = 0.05; %噪声强度 
D = D + randn(n,3) * noiseIntensity;

%% real data
% 
% mag= [
% -17.2500  47.5167  -0.6000 
%  21.1417   8.3333 -22.8083 
%  19.9583  -8.8750 -12.8083 
% -33.9167 -36.7083   9.3750 
% -53.6417   4.7667  17.7250 
% -54.2083  11.1833  12.9333 
% -30.5000  43.3750   3.8083 
% -19.4333  47.2500  -0.1667 
%   6.1250  31.0833 -25.5417 
%   7.7250  12.5583 -32.5833 
%   2.5000 -15.5583 -28.0167 
%  -6.3500 -34.3917 -18.7667 
% -27.0417 -40.0583  12.0167 
% -29.7500 -33.0167  25.2250 
% -28.4583  43.2500  11.9750 
% -14.0583 -36.1833  26.4583 
% -13.7250  16.8500 -35.5000 
% -12.9583  -5.8917 -37.2250 
% -16.9583 -22.6833 -31.0583 
% -21.2917  45.6833  12.9750 
% -18.5583  33.2500 -25.9333 
% -21.7083  -5.3750  43.7500 
% -23.6000 -26.6000 -27.7500 
% -24.8500 -40.2083  -6.8917 
% -12.2500 -42.2250  13.7250 
% -20.2917  47.2083   4.2083 
% -26.6000  41.6667 -11.3750 
% -26.0833  21.9167 -29.7917 
% -32.6417  -9.6667 -30.7917 
% -27.9333 -18.0833 -25.0583 
% -22.2667 -39.0417 -10.6417 
% -10.6417 -43.1417   3.7250 
% -30.3083  42.8333  -2.3333 
% -41.8083  25.1833 -17.8500 
% -50.2667  -7.7667 -17.7250 
% -36.5833 -34.4333   0.2500 
%     ];
% 
% D = mag./1.0;
% % D = D(1:15,:);
% n = length(D);

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

[A,b,expmfs] = magcal(D, 'eye')
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
