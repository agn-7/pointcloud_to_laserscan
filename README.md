# Velodyne LiDAR 3D-points (pointcloud2) to LaserScan message.

This package makes two LaserScan messages from 2 Velodyne rows.


## Topic names:

`/scan` for down-side of Velodyne points in order to providing legs scan.

`/scan_ubg` for up-side of Velodyne points in order to providing torso scan.  


## Usage:
 - Clone this repo, move on `velodyne_to_lrf` branch.
 - `roscore`
 - `rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node`


### TODO:

 - Complete launch files.
 - Make it as parametric and config-able package.
 - Make it Scale-able. 
 - Add TF(s) and separate frames on each topics.