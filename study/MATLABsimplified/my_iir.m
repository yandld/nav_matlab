% Butterworth IIR Filter
srate=1000; % sample rate
npts=2000; %npts in signal
Nyquist=srate/2; %Nyquist frequency
lpf=300; %low-pass frequency
order=5; %filter order
t=[0:npts-1]/srate; %time scale for plot
x=(rand(npts,1)*2)-1; % raw data from -1 to +1

[b,a]=butter(order,lpf/Nyquist); %create filter coefficients

filtered_data=filter(b,a,x); % filter using 'b' and 'a' coefficients

% Calculate FFT
fftpts=npts; % number points in FFT
hpts=fftpts/2; % half number of FFT points
binwidth=srate/fftpts;
f=[0:binwidth:srate-binwidth];
x_fft=abs(fft(x))/hpts; %scaled FFT of original signal
filtered_fft=abs(fft(filtered_data))/hpts; %scaled FFT of filtered signal

subplot(2,2,1)
plot(t,x);
title('Raw Time Series');
subplot(2,2,3)
plot(t,filtered_data);
title('Filtered Time Series');
xlabel('Time (s)');
subplot(2,2,2)
plot(f(1:hpts),x_fft(1:hpts));
title('Raw FFT');
subplot(2,2,4)
plot(f(1:hpts),filtered_fft(1:hpts));
title('Filtered FFT');
xlabel('Frequency (Hz)');
