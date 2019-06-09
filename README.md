# 3D-scanner-ROS

IMPORTANT: 
also see scanner_pcl/Readme.md for more guidlines

## Dependencies:
Complete pack of ROS Melodic and some control staff:

```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

## Quick Start

Gazebo:
```bash
    roslaunch gazebo_ros empty_world.launch 
```

Spawn:
```bash
    roslaunch scanner scanner_control.launch
```

Rviz (errors will hide if run scanner_control.launch):
```bash
    roslaunch scanner scanner_rviz.launch 
```

Moving script:
```bash 
    # rosrun scanner_pcl rotate_and_scan [Num of rotations] [path to config]
    rosrun scanner_pcl rotate_and_scan 20 ~/catkin_ws/srccanner/scanner_pcl/conf/ROTATE_AND_SCAN.json
```

Example of hand by Moving Joints:
```bash
    rostopic pub /scanner/joint2_position_controller/command std_msgs/Float64 "data: -0.9"
```

TODO: 
  * Kinect drivers
  * Add rqt
  * Make parameters computing based on bounding box
  * Return fixed joints to rotatable
  * Path planning
  * Make reconstruction automatic (via meshlab server package)
