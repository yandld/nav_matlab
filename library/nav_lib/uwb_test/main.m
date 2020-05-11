%% clear environment
clc;
close all;
clear;

%%
load('uwb_test_dataset1.mat');

%% remove outliler
tof = dataset.uwb.tof;
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


%% ½âËãÎ»ÖÃ
n = length(tof);

pos = [0.5 0.5]';
dataset.uwb.pos = zeros(2,n);

for i = 1:n
    pos =  ch_multilateration(dataset.uwb.anchor, pos, tof(:,i)');
    dataset.uwb.pos(:,i)   =pos;
end

%% plot data
ch_plot_uwb(dataset.uwb, 2);


