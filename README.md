# Sensor Fusion Nano degree: Project 1

### Lidar Obstacle Detection

Implement Obstacle detection on real point cloud data from a lidar
PointCloudLibrary is used for general data handling and initial testing.
Scope of development includes
- PCD filtering, to reduce computational cost ensuring sufficient data quality
- Segmentation of the filtered cloud into two parts, road and obstacles, using RANSAC based 3D-plane extraction
- Cluster the obstacle cloud, using K-D Tree for 3D space.
- Put bounding boxes for identified clusters in 3D visualization


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php

