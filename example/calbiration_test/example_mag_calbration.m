close all; 
clear;
clc;
format short;

%% simulate data 
% c = [-0.5; 0.2; 0.1]; % ellipsoid center
% r = [1; 1; 2]; % semiaxis radii

% [x,y,z] = ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3),6);
% D = [x(:),y(:),z(:)];
% 
% % add noise:
% n = length(D);
% noiseIntensity = 0.01; %‘Î…˘«ø∂» 
% D = D + randn(n,3) * noiseIntensity;


%% Load data
load('mag_data1.mat');

% Generate simulated ellipsoid data
%[D, true_A, true_b] = generate_simulated_data(500, 0.1, 50);


% Normalize magnetometer data
D = mag;
n = length(D);

% MATLAB internal fitting 
[A, b, expmfs] = magcal(D, 'auto');

% Display calibration results
fprintf('Calibration Matrix:\n');
disp(A);
fprintf('Hard Iron Interference (bias): [%.3f, %.3f, %.3f]\n', b);
fprintf('Expected Magnetic Field Strength: %.3f\n', expmfs);


% Apply calibration
C = (D - b) * A;

% Plot 3D comparison
figure('Name', '3D Comparison');
plotComparison3D(D, C);

% Plot 2D comparisons
figure('Name', '2D Comparisons');
subplot(1, 2, 1);
plotComparison2D(D, C, 1, 3, 'X', 'Z');
subplot(1, 2, 2);
plotComparison2D(D, C, 2, 3, 'Y', 'Z');




%% Helper functions
function plotComparison3D(uncalibrated, calibrated)
    plot3(uncalibrated(:,1), uncalibrated(:,2), uncalibrated(:,3), ...
        'LineStyle', 'none', 'Marker', 'X', 'MarkerSize', 2);
    hold on;
    plot3(calibrated(:,1), calibrated(:,2), calibrated(:,3), ...
        'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'r');
    grid on;
    axis equal;
    xlabel('uT'); ylabel('uT'); zlabel('uT');
    legend('Uncalibrated', 'Calibrated', 'Location', 'southoutside');
    title("Uncalibrated vs Calibrated Magnetometer Measurements");
    hold off;
end

function plotComparison2D(uncalibrated, calibrated, xIndex, yIndex, xLabel, yLabel)
    plot(uncalibrated(:,xIndex), uncalibrated(:,yIndex), ...
        'LineStyle', 'none', 'Marker', 'X', 'MarkerSize', 2);
    hold on;
    plot(calibrated(:,xIndex), calibrated(:,yIndex), ...
        'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'r');
    grid on;
    axis equal;
    xlabel([xLabel '(uT)']); ylabel([yLabel '(uT)']);
    legend('Uncalibrated', 'Calibrated', 'Location', 'southoutside');
    title("Uncalibrated vs Calibrated Magnetometer Measurements");
    hold off;
end

function [D, true_A, true_b] = generate_simulated_data(num_points, noise_level, field_strength)
    % Generate random points on a unit sphere
    phi = 2 * pi * rand(num_points, 1);
    theta = acos(2 * rand(num_points, 1) - 1);
    x = sin(theta) .* cos(phi);
    y = sin(theta) .* sin(phi);
    z = cos(theta);
    D = [x, y, z];

    % Scale to Earth's magnetic field strength
    D = D * field_strength;

    % Define true calibration parameters
    true_A = [1.2, 0.1, -0.1; 0.1, 0.9, 0.05; -0.1, 0.05, 1.1];
    true_b = [16; -30; 2];  % Reduced bias to be more realistic

    % Apply true calibration parameters
    D = (D * true_A') + true_b';

    % Add noise
    noise = noise_level * field_strength * randn(size(D));
    D = D + noise;
end

% %%  elliposid test 1 
% [A, b, expmfs, er]=ellipsoid_fit(D, 5);
% 
% A
% b
% expmfs
% er

