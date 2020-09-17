# Code Setup and Notes

## Download files
```bash
$ git clone https://github.com/willhunt/ottobot.git
```

## Install dependencies
If on Linux Mint,  or getting "unsupported OS error":
```bash
$ export ROS_OS_OVERRIDE=ubuntu:18.04:bionic
```
```bash
$ cd <working directory>/ottobot/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
```

## Build
```bash
$ catkin_make
```

## Run
### Simulation
```sh
$ cd catkin_ws/
$ source devel/setup.bash
$ roslaunch ottobot_gazebo simulation.launch
```
