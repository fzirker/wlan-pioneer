# wlan_pioneer
ROS package to map WLAN availability using a mobile robot (Pioneer P3-DX). The map is created with slam and the wlan signal is displayed as a heatmap.

This is a project for the lecture Autonomous Mobile Robots (AMR) at [Hochschule Mannheim](https://www.hs-mannheim.de/).


## Architecture
### Hardware:
 - Pioneer P3-DX robot
 - Sick laser scanner
 - (Game) Controller
 - WLAN Stick (2.4G and 5G)
 
 ### Software
 - Ubuntu 18.04 alias Bionic Beaver
 - ROS Melodic
 - [catkin](http://wiki.ros.org/catkin)
 - [P2os driver](http://wiki.ros.org/p2os-purdue) ([code from here](https://github.com/allenh1/p2os))
 - [slam_gmapping](http://wiki.ros.org/slam_gmapping) ([code from here](https://github.com/ros-perception/slam_gmapping))
 - [openslam_gmapping](http://wiki.ros.org/openslam_gmapping) ([code from here](https://github.com/ros-perception/openslam_gmapping)))
 - [wifi-scan library](https://github.com/fzirker/wifi-scan)
