srate = 100; % 100 Hz sample rate
speriod = 1/srate; % sample period (s)
dur=1; % duration in seconds
t=[0:speriod:dur-speriod];% Create a signal from 0 to 1 s. The sample period is 0.01 seconds.

freq=2; % frequency of sine wave in Hz
s=sin(2*pi*freq.*t); % calculate sine wave.
plot(t,s); % plot using t as the time base

sfft=fft(s); % calculate Fast Fourier Transform
sfft_mag=abs(sfft); % take abs to return the magnitude of the frequency components
plot(sfft_mag); % plot fft result

fftpts=length(s);
hpts=fftpts/2; % half of the number points in FFT
sfft_mag_scaled=sfft_mag/hpts; % scale FFT result by half of the number of points used in the FFT
plot(sfft_mag_scaled); % plot scaled fft result

binwidth = srate/fftpts; % 100 Hz sample rate/ 100 points in FFT
f=[0:binwidth:srate-binwidth]; % frequency scale goes from 0 to sample rate.Since we are counting from zero, we subtract off one binwidth to get the correct number of points
plot(f,sfft_mag_scaled); % plot fft with frequency axis
xlabel('Frequency (Hz)'); % label x-axis
ylabel('Amplitude'); % label y-axis

%plot first half of data
plot(f(1:hpts),sfft_mag_scaled(1:hpts));
xlabel('Frequency (Hz)');
ylabel('Amplitude');

[magnitude, index]=max(sfft_mag_scaled(1:hpts))



