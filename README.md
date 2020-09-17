# Ottobot
Mobile robot platform using ROS


## Getting Started
#### Clone the package
```sh
$ cd /home/<project folder>/ 
$ git clone https://github.com/willhunt/ottobot.git
```

#### Build the ottobot package
```sh
$ cd catkin_ws/
$ catkin_make
```

#### Make sure to check and install any missing dependencies
```sh
$ export ROS_OS_OVERRIDE=ubuntu:18.04:bionic # Can use this if getting "unsupported OS error", for exampe with Linux Mint
$ rosdep install --from-paths src --ignore-src -r -y
```

#### Install Arduino libraries
Arduino libraries required:
* Rosserial ([see docs](https://willhunt.github.io/ottobot/build-notes/arduino/))
* PID ([PIDLibrary](https://playground.arduino.cc/Code/PIDLibrary/))
* ~~PID Autotune ([PIDAutoTuneLibrary](https://playground.arduino.cc/Code/PIDAutotuneLibrary/))~~

## Run
### Simulation
```sh
$ cd catkin_ws/
$ source devel/setup.bash
$ roslaunch ottobot_gazebo simulation.launch
```

## Documentation
The project site can be viewed here:  
[Github Pages Site](https://willhunt.github.io/ottobot/)  
