close all;
clear;
clc;

format long g;
format compact;

R2D = 180/pi;
D2R = pi/180;
g = 9.8;
Re = 6378137;
Earth_e = 0.00335281066474748;

    % 纯惯性姿态更新
    nQb = [1 0 0 0]';
    pos = [0 0 0]';
    vel = [ 0 0 0]';
    w_b = [0.1 -0.2 -0.3]';
    f_b = [0.1, -0.2, 10]';
    imu_dt = 0.01;
    for i = 1:1000
            [nQb, pos, vel, q] = ins(w_b, f_b, nQb, pos, vel, g, imu_dt);
    end
[p r y] = q2att(nQb');
eul = ([p r y])
nQb
pos
vel




    