%% clear environment
clc;
close all;
clear;

%
% test_line = AA(:,2);
%
% %test_line = [57 59 60 100 59 58 57 58 300 61 62 60 62 58 57];
% t = 1:length(test_line);
%
% %[B,TF,U,L,C] = filloutliers(test_line, 'clip','movmedian',200);
% %B = medfilt1(test_line, 30);
% B = movmean(test_line,20);
%
% plot(t,test_line, '.',t,B,'o')
% legend('Original Data','Filled 4Data')
%
% norm(test_line-B)

load('uwb_test_dataset1.mat');


dataset.uwb.cnt = 3;
dataset.uwb.anchor = [0,0; 2.5, 0; 0, 8]';
n = length(dataset.uwb.tof);


pos = [ 1 0]';
dataset.uwb.pos = zeros(2,n);

for i = 1:n
    pos =  ch_multilateration(dataset.uwb.anchor, pos, dataset.uwb.tof(:,i)');
    dataset.uwb.pos(:,i)   =pos;
end

%% plot data
ch_plot_uwb(dataset.uwb);


