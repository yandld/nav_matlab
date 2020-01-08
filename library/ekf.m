%% Algorithm for Extended Kalman Filter:
% Linearize input functions f and g to get fy(state transition matrix)
% and H(observation matrix) for an ordinary Kalman Filter:
% State Equation:
%     X(n+1) = fy * X(n) + w(n)
% Observation Equation:
%     Z(n) = H * X(n) + v(n)
%
% 1. Xp = f(Xi)                     : One step projection, also provides 
%                                     linearization point
% 
% 2. 
%          d f    |
% fy = -----------|                 : Linearize state equation, fy is the
%          d X    |X=Xp               Jacobian of the process model
%       
% 
% 3.
%          d g    |
% H  = -----------|                 : Linearize observation equation, H is
%          d X    |X=Xp               the Jacobian of the measurement model
%             
%       
% 4. Pp = fy * Pi * fy' + Q         : Covariance of Xp
% 
% 5. K = Pp * H' * inv(H * Pp * H' + R): Kalman Gain
% 
% 6. Xo = Xp + K * (Z - g(Xp))      : Output state
% 
% 7. Po = [I - K * H] * Pp          : Covariance of Xo

%% function
function [X, P] = ekf(X, Z, R, Q, P, FX, GX)
%     EKF  N: number of state, M: number observations 
%
%      Z Measurement signal               observations  M x 1
%     Q: process noise covariance matrix, N x N
%     R: measurement noise covariance matrix, M x M
%     X: "a priori" state estimate, N x 1
%     P: "a priori" estimated state covariance, N x N
%     FX: function for state transition, it takes a state variable Xn and
%       returns 1) f(Xn) and 2) Jacobian of f at Xn(F). As a fake example:
%           function [Val, Jacob] = f(X)
%           Val = sin(X) + 3;
%           Jacob = cos(X);
%           end
%     GX: function for measurement, it takes the state variable Xn and
%       returns 1) g(Xn) and 2) Jacobian of g at Xn.(H)

    n = size(X, 1);    
    I = eye(n, n);
    
    % Predict
    [X, F] = FX(X);
    P = F * P * F'+Q;  
    
    % Update
    [gx, H] = GX(X);
    G = P * H' / (H * P * H.' + R);
    P = (I - G * H) * P;
    X = X + G * (Z - gx);
    
end
