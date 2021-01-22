clc;
clear;

meas  = [pi/3, 0, 5; pi/4, pi/4, 7; pi/6, pi/2, 4; pi/5, 3*pi/4, 6; pi/8, pi, 3];

for i = 1: length(meas)
    p(i,:) = sph_to_cart(meas(i,1), meas(i,2), meas(i,3));
end

param_est = estimate_params(p);
param_est
    
function p = sph_to_cart(epsilon, alpha, r)
% Transform sensor readings to Cartesian coordinates in the sensor frames. 

    p(1) = r * cos(alpha) * cos(epsilon);
    p(2) = r * sin(alpha) * cos(epsilon);
    p(3) = r * sin(epsilon);
end



function param_est = estimate_params(P)
    % Estimate parameters from sensor readings in the Cartesian frame.
    %  Each row in the P matrix contains a single measurement.

    b = P(:,3);
    A = ones(length(P), 3);
    A(:,2:3) = P(:,1:2);
    
    param_est =  inv(A'*A) * A' * b;
    
end
    