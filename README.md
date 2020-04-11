# Ottobot
Mobile robot platform using ROS

[Github Pages Site](https://willhunt.github.io/ottobot/)

## Getting Started
#### Clone the package
```sh
$ cd /home/<project folder>/ 
$ git clone https://github.com/willhunt/ottobot.git
```

#### Build the `robo_nd_project5` package
```sh
$ cd raspberrypi/catkin_ws/
$ catkin_make
```

#### Make sure to check and install any missing dependencies
```sh
$ export ROS_OS_OVERRIDE=ubuntu:18.04:bionic # Can use this if getting "unsupported OS error", for exampe with Linux Mint
$ rosdep install --from-paths src --ignore-src -r -y
```
