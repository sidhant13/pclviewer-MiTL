# pclviewer-t90
The repositiory is built to simulate Man in The Loop(MiTL) display systems. 
The scenario taken is of a LIDAR attached to a seeker kind of missiles. Automatic recognition algorithms in this seeker systems are not optimal enough to provide a good target recognition performance. By introducing a Man In the Loop in this process, where a human can take decisions on target, civilian casulalities can be decreased. 
Lidar attached to an aircraft or seeker missile provides a point cloud of the target environment which is beamed to pilot or ground station.

*###1. res###* 
This folder contains the LIDAR output point cloud files, at different zoom levels to simulate an advancing missile.

2.src
This folder contains the source files. The code is written in C++ utilizing PCL 1.7 for processing, transformation and visualization of Point CLoud.

3.config
This folder contains the property.ini file which lets the man in the loop specify by what colour to code different zoom levels point cloud and to optionally triangualate the point cloud. Greedy Estimation algorithm is used to triangulate point cloud.

4.Screenshot
This folder contains the screenshot and the screencast video of the entire visualization process and features.

Each zoom level is displayed for 1 sec and once the man selects a target by clicking a pint on the point cloud, timer stops and program asks for the confirmation. If the human declines the target, normality resumes till the last zoom level.

References:
MiTL Wiki http://en.wikipedia.org/wiki/Human-in-the-loop
Point Cloud Library http://pointclouds.org/
Trinagulation Algorithm http://pointclouds.org/documentation/tutorials/greedy_projection.php
