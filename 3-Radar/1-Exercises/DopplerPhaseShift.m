% Using the following MATLAB code sample, 
% complete the TODOs to calculate the velocity 
% in m/s of four targets with the following 
% doppler frequency shifts: 
% [3 KHz, -4.5 KHz, 11 KHz, -3 KHz].

clc;
% Doppler Velocity Calculation
c = 3*10^8;         % speed of light
frequency = 77e9;   % frequency in Hz

% TODO: Calculate the wavelength
lambda = c/frequency;
fprintf("Wavelength = %.2f um\n", lambda*1e6);


% TODO: Define the doppler shifts in Hz using the information from above 
Fd = [3e3, -4.5e3, 11e3, -3e3];

% TODO: Calculate the velocity of the targets  fd = 2*vr/lambda
vr = Fd*lambda/2;


% TODO: Display results
fprintf("Target velocity (m/s): "); 
% +vr: approaching vehicle
% -vr: receading vehicle
disp(vr);