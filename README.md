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

## Usage
* TODO

## Contacts
The repository is maintained by Omri Eitan, omrieitan@gmail.com
