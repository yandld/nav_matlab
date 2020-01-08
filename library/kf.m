function [X , P] = kf( X, Z, F, H, R, Q, P)
% Simple Kalman Filter (linear) with optimal gain, no control signal
%
% Z Measurement signal              m observations X # of observations
% F State transition model          n X n, n = # of state values
% H Observation model               m X n
% R Covariance of observation noise m X m
% Q Covariance of process noise     n X n
% P  Ð­·½²î¾ØÕó
% Based on http://en.wikipedia.org/wiki/Kalman_filter, but matrices named
% A, C, G instead of F, H, K.
%
% See http://home.wlu.edu/~levys/kalman_tutorial for background


% n:  # of state values
% m: # of observations

    I = eye(size(F));
   
    % Predict
    X = F * X;
    P = F * P * F' + Q;
    
    % Update
    G = P  * H' / (H * P * H' + R);
    P = (I - G * H) * P;
    X = X + G * (Z - H * X);
end
    
    