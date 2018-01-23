# ROS Package for Handyman

This project is ROS package for the competition of Handyman.

## Prerequisites

- OS: Ubuntu 16.04
- ROS distribution: Kinetic Kame

## How to Install

### Install Rosbridge Server

Please see below.  
http://wiki.ros.org/rosbridge_suite

### Install SIGVerse Rosbridge Server

Please see below.  
https://github.com/SIGVerse/ros_package/tree/master/sigverse_ros_bridge

### Install ROS Package of Handyman

```bash:
$ cd ~/catkin_ws/src
$ git clone https://github.com/PartnerRobotChallengeVirtual/handyman-ros.git
$ cd ..
$ catkin_make
```

## How to Execute

### How to Execute Sample ROS Node

It is a simple ROS node that communicates with the Handyman application.

```bash:
$ roslaunch handyman sample.launch
```

### How to Execute Teleoperation Tool

You can operate HSR with keyboard operation.  
It is for debugging.

```bash:
$ roslaunch handyman teleop_key.launch
```

## License

This project is licensed under the SIGVerse License - see the LICENSE.txt file for details.
