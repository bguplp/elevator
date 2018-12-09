# Armadillo2 - Elevator

## Description
A _catkin_ package, that enables the _Armadillo2_ robot to use elevators.
The package was built on Ubuntu 16.04 LTS in ROS Kinetic with Gazebo 7.14.  
The repository is private and intended to be used by BGU-Armadillo team and Robotican Ltd.

## Pre-requisites
* Ubuntu 16.04 LTS.
* full desktop version of ROS Kinetic - `ros-kinetic-desktop-full`
* armadillo2 in your catkin workspace - [armadillo2 Installation](http://wiki.ros.org/armadillo2/Tutorials/Installation)

## Installation
* clone this repository into `~/catkin_ws/src` and build:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/omrieitan/elevator.git
$ cd ..
$ catkin_make
```
* Create symbolic links of the models to the gazebo models folder:
```
ln -nsf ~/catkin_ws/src/elevator/models/* ~/.gazebo/models
```
* Add the following lines to <armadillo2_moveit_config package>/config/armadillo2_robot.srdf
under the <robot name="armadillo2_robot"> tag:
```xml
<group_state name="button" group="arm">
    <joint name="rotation1_joint" value="0" />
    <joint name="rotation2_joint" value="0" />
    <joint name="shoulder1_joint" value="0" />
    <joint name="shoulder2_joint" value="0" />
    <joint name="shoulder3_joint" value="0.773" />
    <joint name="wrist_joint" value="0" />
</group_state>
```

## Usage
* for simulation run:
```
roslaunch elevator sim_elevator.launch
```
it's also possible to set robot's initial position with these args: x, y, Y

* for real robot run in separate consoles:
```
roslaunch armadillo2 armadillo2.launch lidar:=true move_base:=true moveit:=true intel_cam:=true kinect:=true amcl:=true have_map:=true map:=<path to map yaml file>
roslaunch elevator arm_server.launch
roslaunch elevator elevator.launch 
```
* additional features and options are documented in the launch files and scripts

## Some Notes
* __IMPORTANT__: use the _show_image_ option for debugging only, as it affects the algorithms accuracy and speed
* You may want choose to set a panel image if you want to start button detection from more than 1 meter away
* To maximize detection success rate, use the robot's cameras to take the images and include enough features.
* You may choose to take an image in which the pressing area is not in the center and use the _press_offset_x_ and _press_offset_y_ options to tell the robot where to press
* __useful standalones for other projects__: you may use _arm_server_node_ or _nav_client_. find the documentation in their launch files and scripts.

## Contacts
The repository is maintained by Omri Eitan, omrieitan@gmail.com
