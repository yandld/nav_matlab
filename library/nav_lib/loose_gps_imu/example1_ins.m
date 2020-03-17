%% inertial_na_local_tan

clear;
clc;
close all;

%% settings
dt = 0.01;
Tmax = 600;
t = 0:dt:Tmax;
N = length(t);

% Initialize state vector x
x = zeros(10,1);
x(7:10) = [1 0 0 0];

% set bias
accb = [0.00; 0.00; 0.00];
gyrb = deg2rad([0; 0; 0]);

G = [0, 0, -9.8184]';
u = [G + accb ; gyrb];

p = zeros(3,N);


for n=2:N
    x = ch_nav_equ_local_tan(x, u, dt, G);
    
    p(:,n) = x(1:3);
end

figure(1)
clf
%plot(t,pos)
%plot(t,posL')
plot(t,p)
%plot(t,diff)
grid on
ylabel('Position error [m]')
%ylabel('Difference in position error [m]')
xlabel('Time [s]')
legend('x-axis error','y-axis error','z-axis error')
