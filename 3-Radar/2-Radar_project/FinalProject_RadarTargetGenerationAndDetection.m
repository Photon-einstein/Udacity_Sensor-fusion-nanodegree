clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

rangeMax = 200;             % maximum range (m)
radarRangeResolution = 1;   % radar range resolution (m)
vMax = 70;                  % maximum velocity (m/s)
c = 3.0e8;                  % speed of light (m/s)

% speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
targetRangeInitial = 90;        % target initial position (m)
targetVelocity = 40;            % target velocity (m/s)
                                % +vr: approaching vehicle
                                % -vr: receading vehicle

%% FMCW Waveform Generation

% *%TODO* :
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

% Operating carrier frequency of Radar 
fc= 77e9;                               % carrier freq (Hz)
Bsweep = c/(2*radarRangeResolution);    % sweep bandwith (Hz)
Tchirp = 5.5*2*rangeMax/c;              % sweep time (s)
slope = Bsweep/Tchirp;                  % slope of the FMCW chirp (Hz/s)
fprintf("slope value = %.3e (Hz/T)\n", slope);

% The number of chirps in one sequence. Its ideal to have 2^ value for the 
% ease of running the FFT for Doppler Estimation. 
Nd = 128;                   % #of doppler cells OR #of sent periods % number of chirps

% The number of samples on each chirp. 
Nr = 1024;                  % for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0,Nd*Tchirp,Nr*Nd); % total time for samples


% Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = zeros(1,length(t));      % transmitted signal
Rx = zeros(1,length(t));      % received signal
Mix = zeros(1,length(t));     % beat signal

% Similar vectors for range_covered and time delay.
r_t = zeros(1,length(t));
td = zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % *%TODO* :
    % For each time stamp update the Range of the Target for constant velocity. 
    targetRange = targetRangeInitial + targetVelocity*t(i);
    r_t(i) = targetRange;
    td(i) = (2*targetRange)/c;
    
    % *%TODO* :
    % For each time sample we need update the transmitted and
    % received signal. 
    alpha = slope;
    Tx(i) = cos(2*pi*(fc*t(i)+alpha* (t(i)^2)/2 )); 
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+alpha* ((t(i)-td(i))^2)/2 ));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) =  Tx(i).*Rx(i);
    
end

%% RANGE MEASUREMENT


% *%TODO* :
% reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
% Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);
 
% *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
fft_range = fft(Mix,Nr)./Nr;

% *%TODO* :
% Take the absolute value of FFT output
fft_range = abs(fft_range);

% *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
fft_range = fft_range(1:Nr/2+1);

%plotting the range
figure ('Name','Range from First FFT')
% Get the screen size
screenSize = get(0, 'ScreenSize');

% Set the figure position and size to match the screen size
set(gcf, 'Position', screenSize);

subplot(2,1,1)

% *%TODO* :
% plot FFT output

plot(fft_range)
title('FFT from Range dimension')
xlabel('Range (m)')
ylabel('|fft(m)|')
axis ([0 200 0 0.5]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

% Get the screen size
screenSize = get(0, 'ScreenSize');
% Set the figure position and size to match the screen size
set(gcf, 'Position', screenSize);

title("2D FFT: Range-Doppler Map")
xlabel("Velocity (m/s)")
ylabel("Range (m)");
zlabel("|2D FFT| db")

%% CFAR implementation

% Slide Window through the complete Range Doppler Map

% *%TODO* :
% Select the number of Training Cells in both the dimensions.
Td = 8;
Tr = 8;

% *%TODO* :
% Select the number of Guard Cells in both dimensions around the Cell under 
% test (CUT) for accurate estimation
Gd = 4;
Gr = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 6;

% *%TODO* :
% Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
% design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
% than the Range Doppler Map as the CUT cannot be located at the edges of
% matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

filteredMapping = zeros(Nr/2, Nd); % only half of frequency considered due to simmetry

% for loop that iterate by each row (Range dimension)
for i = (Tr+Gr+1) : (Nr/2-Gr-Tr)
    % for loop that iterate by each column (Doppler dimension)
    for j = (Td+Gd+1) : (Nd-Gd-Td)
        noiseLevelSum = 0;
        numberElementsSum = 0;
        % for loop to sum the training cells in the row dimension
        for r = (i-Tr-Gr) : (i+Gr+Tr)
            % for loop to sum the training cells in the column dimension
            for c = (j-Td-Gd) : (j+Gd+Td)
                if(abs(r-i) > Gr && abs(c-j) > Gd)
                    % update average parameters
                    noiseLevelSum= noiseLevelSum+db2pow(RDM(r,c));
                    numberElementsSum=numberElementsSum+1;
                end
            end % end c
        end % end r

        % averaging the noise
        noiseLevelAverage = noiseLevelSum/numberElementsSum; % linear
        % linear to db scale
        noiseLevelAverageDb = pow2db(noiseLevelAverage); % db
        % add the offset also in db scale
        noiseThreshold_Db = noiseLevelAverageDb + pow2db(offset); % db

        % test Cell Under Test - CUT
        if(RDM(i,j) > noiseThreshold_Db)
            filteredMapping(i,j) = 1;
        end

    end % end j
end % end i


% *%TODO* :
% display the CFAR output using the Surf function like we did for Range
% Doppler Response output.
figure

% Get the screen size
screenSize = get(0, 'ScreenSize');
% Set the figure position and size to match the screen size
set(gcf, 'Position', screenSize);

surf(doppler_axis,range_axis,filteredMapping);
titleStr = "2D CFAR: Constant False Alarm Rate detection"+newline+"on Range-Doppler Map";
title(titleStr)
xlabel("Velocity (m/s)")
ylabel("Range (m)");
zlabel("CFAR binary detection oracle")
colorbar; 