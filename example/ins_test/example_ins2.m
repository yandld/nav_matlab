%% example 惯导解算
clear;
clc;
close all;

%%
Fs = 100;

% GNSS-SIM仿真软件真值
pos_gt = csvread('generated_data/ref_pos.csv', 1, 0);
pos_gt = pos_gt - pos_gt(1,:);
att_gt =  csvread('generated_data/ref_att_euler.csv', 1, 0);
vel_gt =  csvread('generated_data/ref_vel.csv', 1, 0);

% GNSS-SIM 软件仿真结果
pos_py_sim = csvread('generated_data/pos-algo0_0.csv', 1, 0);
pos_py_sim = pos_py_sim - pos_py_sim(1,:);

% m/s^(2), deg/s
 acc = csvread('generated_data/accel-0.csv', 1, 0);
 gyr = csvread('generated_data/gyro-0.csv', 1, 0);
  
gyr = deg2rad(gyr);
N = length(acc);

% 惯导解算
x = zeros(10,1);
x(7:10) = [1 0 0 0];

for i=1:N
    u = [acc(i,:) gyr(i,:)]';
    x = ch_nav_equ_local_tan(x, u , 1 / Fs, [0, 0, 9.795]');
    pos_matlab(i,:) = x(1:3);
    att(i,:) = rad2deg(ch_q2eul(x(7:10)));
    vel(i,:) = x(4:6);
end


ch_plot_pos3d( 'p1', pos_matlab, 'p2', pos_py_sim, 'p3', pos_matlab, 'legend', ["GNSS-SIM真值", "GNSS-SIM仿真结果","matlab仿真结果"]);
ch_plot_pos2d( 'p1', pos_matlab, 'p2', pos_py_sim, 'p3', pos_matlab, 'legend', ["GNSS-SIM真值", "GNSS-SIM仿真结果","matlab仿真结果"]);


fprintf('总时间:%fs  最终位置差:%f\n', N /Fs,  norm(pos_gt(N, :) - pos_matlab(N, :)));
