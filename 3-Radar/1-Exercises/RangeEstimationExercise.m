% Using the following MATLAB code sample, 
% complete the TODOs to calculate the range 
% in meters of four targets with respective 
% measured beat frequencies 
% [0 MHz, 1.1 MHz, 13 MHz, 24 MHz].

clc;

% key parameters defenition
Rmax = 300; % max range of radar in m
rangeRes = 1; % range resolution in m
c = 3.0e8;

% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = c/(2*rangeRes);
fprintf("Bsweep = %.1f MHz\n", Bsweep/1e6);

% TODO : Calculate the chirp time based on the Radar's Max Range
Ts = 5.5*2*Rmax/c;
fprintf("T chirp = %.2f us\n", Ts*10^6);

% TODO : define the frequency shifts 
fBeatArray = [0, 1.1e6, 13e6, 24e6]; % MHz

R = fBeatArray*c*Ts/(2*Bsweep); % range of targets

fprintf("Calculated range (m): ");
calculated_range = R;
% Display the calculated range
disp(calculated_range);