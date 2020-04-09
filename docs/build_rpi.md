---
layout: page
title: Arduino
permalink: /build/rpi/
---

# Notes for using Raspberry Pi with ROS

## Installation
Install Ubuntu Mate: [The Robotics Back-End](https://roboticsbackend.com/install-ubuntu-mate-18-04-on-raspberry-pi-3-b/)

Install ROS: [The Robotics Back-End](https://roboticsbackend.com/install-ros-on-raspberry-pi-3/)

### Additional steps
```bash
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ 
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep update
```

## Interface
Connect to raspberry pi via ssh:
```bash
$ ssh <username>@<ip address>
```
In my case:
```bash
$ ssh will@192.168.0.16
```

Shutdown
```bash
$ sudo shutdown now
```

## Catkin Build
Building large nodes with all cores can cause issues so specifying 2 can be faster and more reliable:
```bash
$ catkin_make -j2
```