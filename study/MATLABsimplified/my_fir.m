npts=1000;
ntaps = 5;
b =ones(1, ntaps) / ntaps; % create filter coefficients for 5- point moving average

x=(rand(npts,1)*2)-1; % raw data from -1 to +1
filtered_data=filter(b,1,x); % filter using 'b' coefficients

subplot(2,1,1); % 1st subplot
plot(x); % plot raw data
title('Raw Data');
subplot(2,1,2); % 2nd subplot
plot(filtered_data); %plot filtered data
title('Filtered Data');
xlabel('Time');


% Perform FFT on original and filtered signal
fftpts=npts; % number points in FFT
hpts=fftpts/2; % half number of FFT points
x_fft=abs(fft(x))/hpts; %scaled FFT of original signal
filtered_fft=abs(fft(filtered_data))/hpts; %scaled FFT of filtered signal

subplot(2,1,1) %1st subplot
plot(x_fft(1:hpts)); %plot first half of data points
title('Raw Data');
subplot(2,1,2) %2nd subplot
plot(filtered_fft(1:hpts));%plot first half data points
title('Filtered Data');
xlabel('Frequency');
