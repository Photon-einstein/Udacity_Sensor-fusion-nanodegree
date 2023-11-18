# Udacity_Sensor-fusion-nanodegree

## Summary
This is a course that summarizes the essential principles of LiDAR, Camera, Radar and Sensor Fusion. Since each sensor has their inherent strengths and limitations, it is important to investigate how they can complement each other to provide the most reliable results when attempting to determine the position and velocity of obstacles.

**Note**: 
These projects are developed in Ubuntu 18.04, ROS Melodic, PCL 1.8 and OpenCV2. Please clone the master branch for latest version of all projects. All projects are compiled using cmake (https://cmake.org/).

## LiDAR Segment - Point Cloud Library
The Random Sample Consensus (RANSAC) principle was used to segment the ground plane from point cloud data. Obstacle Clustering was performed using recursion, with improved efficiency using the Kd-tree data structure.

The main principles taught in this segment are:

1. Plane Segmentation
2. Euclidean Clustering using kd-tree
3. Filtering techniques
4. Reading and streaming PCDs

