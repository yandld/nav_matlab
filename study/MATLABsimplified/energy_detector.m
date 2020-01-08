% Create Noise Signal and embed sine wave in it at a random location
npts=5000; % # points in signal
srate=1000; % sample rate (Hz)
dur=npts/srate; % signal duration in sec
amp_n=3.5; % noise amplitude
amp_t=1.0; % sine wave amplitude
freq=100; % sine wave frequency
sinepts=400; % # points in sine wave

t=[0:npts-1]/srate; % signal time base
sine_t=[0:sinepts-1]/srate; % sine wave time base
noise=(rand(1,npts)-.5)*amp_n; %noise signal
sinewave=amp_t*(sin(2*pi*freq*sine_t)); % sine wave
% random location of sinewave
sinepos=floor(rand(1,1)*(npts-sinepts))+1;
signal = noise; % make signal equal to noise
endsine = sinepos + sinepts-1; %calc index end of sine wave
% add sinewave to signal starting at index sinepos
signal(sinepos:endsine) = signal(sinepos:endsine) + sinewave;

% Plot signal
subplot(5,1,1)
plot(t,signal);
hold on
%plot red dot on location of sine wave start
plot(sinepos/srate,0,'.r');
hold off
title('Original Signal');


% Step 1: Filter signal
hbandwidth=5; %half bandwidth
nyquist=srate/2;
ntaps=200;
hpf=(freq-hbandwidth)/nyquist;
lpf=(freq+hbandwidth)/nyquist;
[b,a]=fir1(ntaps, [hpf lpf]); %calc filter coefficients
signal_f=filter(b,a,signal); %filter signal
subplot(5,1,2)
plot(t,signal_f); %plot filtered signal
hold on
%plot red dot on location of sine wave
plot(sinepos/srate,0,'.r');
hold off
title('Step 1. Filter Signal');



signal_r=abs(signal_f); %abs value of filtered signal
subplot(5,1,3)
plot(t,signal_r); %plot filtered signal
hold on
%plot red dot on location of sine wave
plot(sinepos/srate,0,'.r');
hold off
title('Step 2. Rectify Signal');


% Step 3: Low-pass filter rectified signal
lpf=10/nyquist; % low-pass filter corner frequency
npoles=6;
[b,a]=butter(npoles, lpf); % Butterworth filter coeff
signal_env=filter(b,a,signal_r); %Filter signal
subplot(5,1,4)
plot(t,signal_env); %plot filtered signal
hold on
%plot red dot on location of sine wave
plot(sinepos/srate,0,'.r');
hold off
title('Step 3. Envelope Signal');



% Step 4: Threshold signal
threshold=amp_t/2;
gated=(signal_env>threshold);
subplot(5,1,5)
plot(t,signal_env); %plot filtered signal
hold on
%plot red dot on location of sine wave
plot(sinepos/srate,0,'.r');
plot(t, gated, 'r'); % plot threshold signal
hold off
title('Step 4. Threshold Signal');
xlabel('Time (s)');


d_gated=diff(gated);
plot(d_gated);

find(d_gated==1)

find(d_gated == -1)

abs(find(d_gated==1) - find(d_gated == -1))



