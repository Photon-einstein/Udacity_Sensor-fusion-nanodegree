# Udacity_Sensor-fusion-nanodegree

## Summary
This is a course that summarizes the essential principles of LiDAR, Camera, Radar and Sensor Fusion. Since each sensor has their 
inherent strengths and limitations, it is important to investigate how they can complement each other to provide the most reliable 
results when attempting to determine the position and velocity of obstacles.

**Note**: 
Please clone the master branch for latest version of all projects. All projects are compiled using cmake (https://cmake.org/).

## 1)     LiDAR Segment - Point Cloud Library
The Random Sample Consensus (RANSAC) principle was used to segment the ground plane from point cloud data. Obstacle Clustering was 
performed using recursion, with improved efficiency using the Kd-tree data structure.

The main principles taught in this segment are:

1. Plane Segmentation
2. Euclidean Clustering using kd-tree
3. Filtering techniques
4. Reading and streaming PCDs

The final result is shown below, where the green points represent the street surface and the obstacles are marked in the red boxes.

![Lidar_Obstacle_detection_program_running](https://github.com/Photon-einstein/Udacity_Sensor-fusion-nanodegree/assets/31144077/c3b96c60-a25d-43d3-909d-c4f8c851ef4e)

## 2)     3D Object Tracking based on Camera

This project tracks the preceding vehicle in the same lane and estimates the time-to-collision (TTC) based on both camera images and Lidar data. 
The construction of the camera TTC estimation pipeline, demanded the implementation of the keypoint detection, descriptor extraction, and methods 
matched keypoints between successive images. 
With the help of 3D bounding boxes, I was able to extract keypoints corresponding to the preceding vehicle and calculate the TTC based on relative 
distance between matched keypoints in two successive images. Matched keypoints also contributed to match 3D bounding boxes in the Lidar point cloud, 
so that Lidar pipeline could estimate the TTC using the closest distances of the bounding boxes to the ego vehicle in two successive frame.

The output short video is shown below, where the preceding vehicle is tracked with a green box, 3D Lidar points on the vehicle trunk are projected to 
the 2D frame (green points). TTC estimations based on Lidar and camera are reported on the top.  

The final result of this project is the estimation of the time to collision to the nearest vehicle, for the Camera and the LIDAR systems written on
top of the frame captured by our vehicle.

![Camera_3D_Object_Tracking_program_running](https://github.com/Photon-einstein/Udacity_Sensor-fusion-nanodegree/assets/31144077/344b50a4-5fa2-4ff8-8905-cb149ec38565)

## 3)      Radar

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


