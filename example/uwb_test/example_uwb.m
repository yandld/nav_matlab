clc;
close all;
clear;

%% load data
load('uwb_test_dataset1.mat');
uwb = dataset.uwb;
%uwb.anchor(3,2) = 1;
tof = uwb.tof;
% f_tof = smoothdata(tof,'rlowess', 100);

%% 解算位置
N = length(tof);

pos = [1 1 1]';
uwb.pos = zeros(size(uwb.anchor,1), N);

% 多边定位解算
for i = 1:N
    pos =  ch_multilateration(uwb.anchor, pos, tof(:,i)', 2);
    uwb.pos(:,i)   =pos;
end

%% plot data

figure;
plot(tof', '.');
title("伪距")

ch_plot_pos3d('p1', uwb.pos',  'legend', ["UWB轨迹"]);
anch = uwb.anchor;
hold all;
scatter(anch(1, :),anch(2, :),'k');
for i=1:size(anch, 2)
    text(anch(1, i),anch(2, i),"A"+(i-1))
end
hold off;



