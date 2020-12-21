clc;
close all;
clear;

%% load data
load('uwb_test_dataset1.mat');
uwb = dataset.uwb;

%% remove outliler
tof = uwb.tof';
f_tof = smoothdata(tof,'rlowess', 100);

figure;
plot(tof, '.');
hold on;
plot(f_tof);
legend('原始X', '原始Y', '原始Z', '滤波X', '滤波Y', '滤波Z');


tof = f_tof';

%% 解算位置
n = length(tof);

pos = [1 1 1]';
uwb.pos = zeros(size(uwb.anchor,1), n);

% 多边定位解算
for i = 1:n
    pos =  ch_multilateration(uwb.anchor, pos, tof(:,i)', 3);
    uwb.pos(:,i)   =pos;
end

%% plot data
ch_plot_uwb(uwb.anchor, uwb.pos, 3);


