# Radar

This project uses radar to generate a perception for a self-driving car, example of covered topics:
-   Frequency Modulated Continuous Wave (FMCW);
-   Signal propagation;
-   Target cinematic; 
-   Response generation;
-   FFT 1D;
-   FFT 2D;
-   Range & Doppler generation;
-   2D CFAR (Constant False Alarm Rate);
-   Simulation in Matlab;

### I. Preparation on Linux

Radar exercises and projects will be completed using MATLAB. It is required to register a MathWorks account [here](https://www.mathworks.com/mwaccount/register). Verify the email and log on the account, then download the installer for Ubuntu [here](https://www.mathworks.com/licensecenter/classroom/udacity_sf_radar/). If the license expiration date is ending soon, we can contact Udacity for an extension [support@udacity.com](support@udacity.com).

Unzip the `matlab_R2023b_glnxa64.zip` and run the installation script with sudo.
```bash
$ sudo ./install
$ ./matlab
```

Tutorial for MATLAB is available [here](https://www.mathworks.com/learn/tutorials/matlab-onramp.html).

### II. Knowledge build up with pratical exercises

[Radar Target Generation and Detection exercises](./1-Exercises)

### II. Project

[Radar Target Generation and Detection](./2-Radar_project/README.md)

### Radar

This project focuses on developing a radar system using sensor fusion techniques. The goal is to detect and track objects in the environment using data from radar sensors. By combining the measurements from multiple radar sensors, we can improve the accuracy and reliability of object detection and tracking.

I was used the Fast Fourier Transform 2D  on the data obtrained from the Frequency Modulated Continuous Wave (FMCW) Radar to obtain the Range-Doppler Map. After that, the Cell Averaging Constant Fast Alarm Rate (CA - CFAR) was conducted to dynamically filter out noise and to retrieve the peak corresponding to the obstacle, giving the range and the velocity estimation. 

From the image below, one can deduce that the obstacle has a displacement of 90m and a velocity of 40m/s.

The main principles taught in this segment are:

-   Frequency Modulated Continuous Wave (FMCW);
-   Signal propagation;
-   Target cinematic; 
-   Response generation;
-   FFT 1D;
-   FFT 2D;
-   Range & Doppler generation;
-   2D CFAR (Constant False Alarm Rate);
-   Simulation in Matlab;