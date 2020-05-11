%% clear environment
clc;
close all;
clear;

%% load data
load('uwb_test_dataset1.mat');
uwb = dataset.uwb;

%% remove outliler
tof = uwb.tof;
t = 1:length(tof);
line = tof(:,:)';
old = line;

for i = 1:4
    d = diff(line);
    outliter =  find(abs(d)>0.1);
    
    for n= 1:length(outliter)
        j = 1;
        while( abs(line(outliter(n)) -  line(outliter(n)-j)) <0.1)
            j =j +1;
        end
          line(outliter(n)) = mean( line(outliter(n)-j));
    end
end

norm(old - line)
% figure;
% plot(line, '.');

tof = line';

uwb.anchor = [uwb.anchor;[1 1 1]];

%% ½âËãÎ»ÖÃ
n = length(tof);

pos = [0.5 0.5, 0.9]';
uwb.pos = zeros(size(uwb.anchor,1),n);

for i = 1:n
    pos =  ch_multilateration(uwb.anchor, pos, tof(:,i)', 3);
    uwb.pos(:,i)   =pos ;
end

%% plot data
ch_plot_uwb(uwb, 3);


