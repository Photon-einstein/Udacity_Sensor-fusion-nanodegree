# Udacity_Sensor-fusion-nanodegree

## Summary
This is a course that summarizes the essential principles of LiDAR, Camera, Radar and Sensor Fusion. Since each sensor has their inherent strengths and limitations, it is important to investigate how they can complement each other to provide the most reliable results when attempting to determine the position and velocity of obstacles.

**Note**: 
Please clone the master branch for latest version of all projects. All projects are compiled using cmake (https://cmake.org/).

## LiDAR Segment - Point Cloud Library
The Random Sample Consensus (RANSAC) principle was used to segment the ground plane from point cloud data. Obstacle Clustering was performed using recursion, with improved efficiency using the Kd-tree data structure.

The main principles taught in this segment are:

1. Plane Segmentation
2. Euclidean Clustering using kd-tree
3. Filtering techniques
4. Reading and streaming PCDs

My final result is shown below, where the green points represent the street surface and the obstacles are marked in the red boxes.

![Lidar_Obstacle_detection_program_running](https://github.com/Photon-einstein/Udacity_Sensor-fusion-nanodegree/assets/31144077/c3b96c60-a25d-43d3-909d-c4f8c851ef4e)

