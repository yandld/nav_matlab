clear;
clc
close all;

addpath('../../library/nav_lib');

%% INPUTS: INITIAL CONDITION

true_user_states = [4245849 -2451342 4113840, 1000000]';

station = zeros(5,3);
station(1,:) = [21630742.37 -7872946.37 13290000];
station(2,:) = [9799722.428 -11678854.4 21773061.34];
station(3,:) = [15014045.82 2647381.37 21773061.34];
station(4,:) = [17020279.96 -20283979.8 2316599.642];
station(5,:) = [26076581.77 4598004.93 2316599.642];
station = station';
x = zeros(4,1);

persdo_range = vecnorm(station - true_user_states(1:3))+true_user_states(4) ;

for i = 1:5
    range = vecnorm(station - x(1:3));
    H(1,1:3) = -(station(:,1) - x(1:3))/range(1);
    H(2,1:3) = -(station(:,2) -  x(1:3))/range(2);
    H(3,1:3) = -(station(:,3) -  x(1:3))/range(3);
    H(4,1:3) = -(station(:,4) -  x(1:3))/range(4);
    H(5,1:3) = -(station(:,5) -  x(1:3))/range(5);
    H = [H(:,1:3),  [1 1 1 1 1]'];
    
    b= ((persdo_range - range) - x(4))';
    
    x = x + inv(H'*H)*H'*b;
    
end







