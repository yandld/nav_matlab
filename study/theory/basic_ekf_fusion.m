clear all; clc;

%A Double-Stage Kalman Filter for Orientation Tracking With an Integrated Processor in 9-D IMU

syms q0 q1 q2 q3 real;



q = [q0 q1 q2 q3];

Rbn = [q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2); 2*(q1*q2 - q0*q3), q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 + q0*q1); 2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0^2 - q1^2 - q2^2 + q3^2];

% acc correct
h1 = Rbn*[0 0 1]';
H1 = jacobian(h1, [q0 q1 q2 q3]);


h1

H1

% mag correct
h2 = Rbn*[0 1 0]';
H2 = jacobian(h2, [q0 q1 q2 q3]);

h2

H2
