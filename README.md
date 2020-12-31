# oslsim

[![OSL](https://img.shields.io/badge/UMich-OSL-yellow)](https://opensourceleg.com/)
[![ROS](https://img.shields.io/badge/ROS-Melodic-blue)](http://wiki.ros.org/melodic)
[![License](https://img.shields.io/github/license/homebrew-bionics/oslsim)](https://github.com/homebrew-bionics/oslsim/blob/master/LICENSE.md)

<img src="./oslsim_banner.gif" width="840">

A ROS package that provides the necessary interfaces to simulate the Open-source leg (OSL) proposed by the **Neurobionics Lab at the University of Michigan** to unify the research field of prosthetic leg controls. OSL is a robust and relatively inexpensive system that can be easily manufactured, assembled, and controlled. OSL is licensed under a [Creative Commons Attribution 3.0 Unported License](https://creativecommons.org/licenses/by/3.0/deed.en_US). To know more, visit https://opensourceleg.com

## Getting Started

<img src="./oslsim.jpg" width="840">

This package requires an installation of [ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/).
```
$ sudo apt-get install ros-<ros-distro>-desktop-full
$ sudo apt-get install ros-<ros-distro>-ros-control
$ sudo apt-get install ros-melodic-effort-controllers
```
*Preferred **\<ros-distro\>**: ``melodic``*. If you are new to **ROS**, read more about [Configuring your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Build the package in your catkin workspace.
```
$ cd <catkin-workspace>/src
$ git clone https://github.com/homebrew-bionics/oslsim.git
$ cd ../
$ catkin_make
$ source devel/setup.bash
```
## Sensors

<img src="./oslsim_imu.gif" width="840">

The Open-source Leg (OSL) model depicted in this package comprises

* 2 Inertial Measurement Units (IMU)
* 2 Rotary encoders
* a Load cell

## Topics
<img src="./topics.png" width="840">

## Load cells

``osl_foot`` has a bumper plugin that acts as a load cell. The plugin publishes
* Force (fx, fy, fz)
* Torque (Tx, Ty, Tz)
* Contact positions
* Contact normals

<img src="./oslsim_loadcell.gif" width="840">

A python script is included within this package that subscribes to the above sensor data and publishes the required ones to any desired ROS topic.

## Run

<p align="center">
    <img src="./rosgraph.png" width="600" class="center">
</p>

To launch the simulation:

```
$ roslaunch oslsim main.launch control:=true
```
<img src="./oslsim_walk.gif" width="840">

The ``oslsim_controller`` node serves as a container to various control strategies and publishes joint commands to their respective position controllers (``osl_knee`` and ``osl_ankle``). 

## Issues and Feature requests
Please report bugs or request features [here](https://github.com/homebrew-bionics/oslsim/issues).