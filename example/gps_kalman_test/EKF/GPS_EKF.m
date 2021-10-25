% Example:
% Kalman filter for GPS positioning
% This file provide an example of using the Extended_KF function with the
% the application of GPS navigation. The pseudorange and satellite position
% of a GPS receiver at fixed location for a period of 25 seconds is
% provided. Least squares and Extended KF are used for this task.
%
% The following is a brief illustration of the principles of GPS. For more
% information see reference [2].
% The Global Positioning System(GPS) is a satellite-based navigation system
% that provides a user with proper equipment access to positioning
% information. The most commonly used approaches for GPS positioning are
% the Iterative Least Square(ILS) and the Kalman filtering(KF) methods.
% Both of them is based on the pseudorange equation:
%                rho = || Xs - X || + b + v
% in which Xs and X represent the position of the satellite and
% receiver, respectively, and || Xs - X || represents the distance between
% them. b represents the clock bias of receiver, and it need to be solved
% along with the position of receiver. rho is a measurement given by
% receiver for each satellites, and v is the pseudorange measurement noise
% modeled as white noise.
% There are 4 unknowns: the coordinate of receiver position X and the clock
% bias b. The ILS can be used to calculate these unknowns and is
% implemented in this extample as a comparison. In the KF solution we use
% the Extended Kalman filter (EKF) to deal with the nonlinearity of the
% pseudorange equation, and a CV model (constant velocity)[1] as the process
% model.

% References:
% 1. R G Brown, P Y C Hwang, "Introduction to random signals and applied
%   Kalman filtering : with MATLAB exercises and solutions",1996
% 2. Pratap Misra, Per Enge, "Global Positioning System Signals,
%   Measurements, and Performance(Second Edition)",2006

clear;
close all;
clc;

load SV_Pos                         % position of satellites
load SV_Rho                         % pseudorange of satellites

T = 1; % positioning interval
N = length(SV_Pos);% total number of steps
% State vector is as [x Vx y Vy z Vz b d].', i.e. the coordinate (x,y,z),
% the clock bias b, and their derivatives.

% Set f, see [1]

% Set Q, see [1]
Sf = 36;
Sg = 0.01;
sigma = 5;         %state transition variance
Qb = [Sf*T+Sg*T*T*T/3 Sg*T*T/2; Sg*T*T/2 Sg*T];

Qxyz = sigma^2 * [T^3/3 T^2/2;  T^2/2 T];

Q = blkdiag(Qxyz, Qxyz, Qxyz, Qb);

% Set initial values of X and P
X = zeros(8,1);
X([1 3 5]) = [ -2168816.18127156 4386648.54909167 4077161.59642875];  %Initial position
X([2 4 6]) = [0 0 0];                                            %Initial velocity
X(7,1) = 3575261.15370644;                                 %Initial clock bias
X(8,1) =  45.4924634584581;                                 %Initial clock drift
P = eye(8)*10;

% Set R
Rhoerror = 36;                 % variance of measurement error(pseudorange error)
R = eye(size(SV_Pos{1}, 1)) * Rhoerror;

fprintf('GPS positioning using EKF started\n')
tic

% leaest squre solutiion
x_ls= [0,0,0,0]';



for i = 1:N
    
    
    % Set Z
    Z = SV_Rho{i}.';           % measurement value
    
    X = state_equ(X, T);
    
    [F, G] = state_propagation(X, T);
    
    %Time update of the Kalman filter state covariance.
    P = F*P*F' + G*Q*G';
    
    % measument update
    [Val, H] = measurement_gps(X, SV_Pos{i});

    % Calculate the Kalman filter gain.
    K=(P*H')/(H*P*H'+R);
    
    % update state
    X = X + K*(Z - Val);
    
    % Update the Kalman filter state covariance.
    P=(eye(length(X))-K*H)*P;

    %% log data
    
    % positioning using Kalman Filter
    Pos_KF(:,i) = X([1 3 5]).';
    
    % positioning using Least Square as a contrast
    Pos_LS(:,i) =  ch_gpsls(x_ls, SV_Pos{i}',  SV_Rho{i});
    
    fprintf('KF time point %d in %d  ',i, N)
    
    time = toc;
    remaintime = time * N / i - time;
       fprintf('Time elapsed: %f seconds, Time remaining: %f seconds\n',time,remaintime)
end

% Plot the results. Relative error is used (i.e. just subtract the mean)
% since we don't have ground truth.
for i = 1:3
    subplot(3,1,i)
    plot(1:N, Pos_KF(i,:) - mean(Pos_KF(i,:)),'-r')
    hold on;grid on;
    plot(1:N, Pos_LS(i,:) - mean(Pos_KF(i,:)))
    legend('EKF','ILS')
    xlabel('Sampling index')
    ylabel('Error(meters)')
end
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Relative positioning error in x,y and z directions','HorizontalAlignment','center','VerticalAlignment', 'top');

%% state equation
function x = state_equ(x, dt)
Jacob = [1,dt; 0,1];
F = blkdiag(Jacob, Jacob, Jacob, Jacob);
x = F*x;
end

%% sate propagation
function [F, G] = state_propagation(x, dt)
Jacob = [1,dt; 0,1];
F = blkdiag(Jacob, Jacob, Jacob, Jacob);
G = eye(length(x));
end

%% measurement equation
function [Val, H] = measurement_gps(X, SV)

% Each row of SV is the coordinate of a satellite.
dX = bsxfun(@minus, X([1,3,5])', SV);% X - Xs
Val = sum(dX .^2, 2) .^0.5 + X(7);
H = zeros(size(SV, 1), size(X, 1));
H(:, [1,3,5]) = bsxfun(@rdivide, dX, Val);
H(:, 7) = 1;

end

