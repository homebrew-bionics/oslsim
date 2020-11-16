# oslsim

[![OSL](https://img.shields.io/badge/UMich-OSL-yellow)](https://opensourceleg.com/)
[![ROS](https://img.shields.io/badge/ROS-Melodic-blue)](http://wiki.ros.org/melodic)
[![License](https://img.shields.io/github/license/homebrew-bionics/oslsim)](https://github.com/homebrew-bionics/oslsim/blob/master/LICENSE.md)

A ROS package that provides the necessary interfaces to simulate the Open-source leg (OSL) proposed by the **Neurobionics Lab at UM** to unify the research field of prosthetic leg controls. OSL is a robust and relatively inexpensive system that can be easily manufactured, assembled, and controlled. OSL is licensed under a [Creative Commons Attribution 3.0 Unported License](https://creativecommons.org/licenses/by/3.0/deed.en_US).

![oslsim](./oslsim.jpg)

To know more, visit https://opensourceleg.com/

## Getting Started
```
$ git clone https://github.com/homebrew-bionics/oslsim.git
$ cd ../
$ catkin_make
$ source devel/setup.bash
```

## Usage
```
$ roslaunch oslsim main.launch walk:=true
```
*Place your controller logic within the ``oslsim_walker`` node.*

![oslsim_walk](./oslsim_walk.jpg)
