clc;
%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Ps = 3e-3;
Ps_dBm = 10*log10(Ps*1000);

%Antenna Gain (linear)
G =  10000;
G_dBi = 10*log10(G);

%Minimum Detectable Power
Pe = 1e-10;
Pe_dB = 10*log10(Pe*1000);

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength/T = c <=> lambda.f = c <=> lambda=c/f
lambda = c/fc;

fprintf("wavelenght = %.2f mm\n", lambda*1000);

%TODO : Measure the Maximum Range a Radar can see.

R_max = nthroot( (Ps*G^2*lambda^2*RCS) / (Pe*(4*pi)^3 ), 4);
fprintf("Range max = %.2f m\n", R_max);


