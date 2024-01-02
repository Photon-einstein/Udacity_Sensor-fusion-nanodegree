% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;
clc;
% Generate Noisy Signal

% Specify the parameters of a signal with a sampling frequency of 1 kHz 
% and a signal duration of 1.5 seconds.

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal, 1.5s
t = (0:L-1)*T;        % Time vector

% Form a signal containing a 50 Hz sinusoid of amplitude 0.7 and a 120 Hz 
% sinusoid of amplitude 1.

S = 0.7*sin(2*pi*50*t) + sin(2*pi*120*t);

% Corrupt the signal with zero-mean white noise with a variance of 4
X = S + 2*randn(size(t));

X_cfar = abs(X);

% Data_points
Ns = 1500;  % let it be the same as the length of the signal

%Targets location. Assigning bin 100, 200, 300, and 700 as Targets
%  with the amplitudes of 16, 18, 27, 22.
X_cfar([100 ,200, 300, 700])=[16 18 27 22];

% plot the output
figure(1);
tiledlayout(2,1)
nexttile
plot(X_cfar)

% Apply CFAR to detect the targets by filtering the noise.

% TODO: Define the number of Training Cells (must be even)
T = 12;
% TODO: Define the number of Guard Cells (must be even)
G = 4;
% TODO: Define Offset (Adding room above noise threshold for the desired SNR)
offset = 6;

% Initialize vector to hold threshold values 
threshold_cfar = zeros(Ns-(G+T+1),1);

% Initialize Vector to hold final signal after thresholding
signal_cfar = zeros(Ns-(G+T+1),1);

% Slide window across the signal length
for i = 1:(Ns-(G+T+1))     
    
    % TODO: Determine the noise threshold by measuring it within 
    % the training cells
    leading_training_cells = X_cfar(i : i+ T/2 -1);
    trailing_training_cells = X_cfar(i+(G + T/2 + 1) : i+(G + T));
    training_cells = vertcat(leading_training_cells, trailing_training_cells);

    noise_level = mean(training_cells, "all");

    % TODO: scale the noise_level by appropriate offset value and take
    % average over T training cells
    threshold = noise_level * offset;

    % Add threshold value to the threshold_cfar vector
    %threshold_cfar = [threshold_cfar, {threshold}];
    threshold_cfar(i) = threshold;

    % TODO: Measure the signal within the CUT
    signal = X_cfar(i+T/2+G/2);
    if signal < threshold
        signal = 0;
    end

    % add signal value to the signal_cfar vector
    signal_cfar(i) = signal;
end

% plot original sig, threshold and filtered signal within the same figure.
nexttile
plot(X_cfar);
hold on
plot(circshift(threshold_cfar,G),'r--','LineWidth',2)
hold on
plot (circshift(signal_cfar,(T+G)),'g--','LineWidth',2);
legend('Signal','CFAR Threshold','detection')