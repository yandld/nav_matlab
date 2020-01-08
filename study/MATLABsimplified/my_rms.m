function h=rms(data)
% h=rms(data)

%1
data_squrared = data.*data;

%2
mean_ds = mean(data_squrared);

%3
h = sqrt(mean_ds);