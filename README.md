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
* Add the following line to <armadillo2_moveit_config package>/config/armadillo2_robot.srdf
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

* make sure you have manipulator_servers package in your `~/catkin_ws/src`, you can clone and build it as follow:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/robotican/manipulator_servers.git
$ cd ..
$ catkin_make
```

## Usage
* for stand alone demo run:
```
roslaunch elevator sim_elevator.launch
```
* to run as part of another algorithm:
```
roslaunch elevator sim_elevator.launch stand_alone:=false
```
and don't forget to set the images.
stand_alone:=false assume you've already launched armadillos.

## Contacts
The repository is maintained by Omri Eitan, omrieitan@gmail.com
