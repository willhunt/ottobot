---
title: Arduino
description: Arduino setup using the M0 Pro.
---

# Notes for using Arduino with ROS

## Setup Notes
### ROS Libraries
Install ros libraries
```bash
$ sudo apt-get install ros-indigo-rosserial-arduino
$ sudo apt-get install ros-indigo-rosserial
```
### Arduino Libraries
The rosserial Arduino library does not support the M0 Pro so a few changes have to be made. Install the latest version from [github](https://github.com/ros-drivers/rosserial) via the instruction on the ROS [wiki](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). Then replace the file **'/rol_lib/ArduinoHardware.h'** with [ArduinoHardware.h](https://github.com/willhunt/ottobot/blob/master/arduino/reference/ArduinoHardware.h). In summary of the changes, some cases have been added to force the serial definitions based upon use of the M0 Pro:

```cpp
...
#elif defined(USE_M0PRO_NATIVE)
    #define SERIAL_CLASS Serial_
#elif defined(USE_M0PRO_PROGRAMMING)
    #include <HardwareSerial.h> // Arduino AVR
    #define SERIAL_CLASS HardwareSerial
...
#elif defined(USE_M0PRO_NATIVE)
            iostream = &SerialUSB;
...
```

In the Arduino sketch file it must have definitions:
```cpp
#define USE_USBCON
// Use this for Native USB port
#define USE_M0PRO_NATIVE
// Use this for Programming USB port
// #define USE_M0PRO_PROGRAMMING
```

## Custom Messages
To use custom messages with Arduino, first [create the custom message](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv), then make sure ros_serial is installed. This can be done by adding it as a build dependency in the relevant `package.xml` file:
```xml
<build_depend>rosserial_arduino</build_depend>
```
and installing dependencies for the project:
```bash
$ rosdep install --from-paths src --ignore-src -r -y
```

Then from your catkin workspace run:
```bash
$ source devel/setup.bash
$ rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```
This will create the `ros_lib` arduino library folder in the arduino library directory. Any desired location can be specified.

A script has been created to automate this task including copying of the modified `ArduinoHardware.h` file (see [Arduino Libraries](./arduino.md#arduino-libraries) section).
```bash
$ cd arduino/scripts
$ sudo chmod +x make_arduino_libraries.sh
$ ./make_arduino_libraries.sh
```

## ROS Interfacing
### Port
Find the Arduino port using:
```bash
$ ls /dev/ttyACM*
```
Or possibly:
```bash
$ ls /dev/ttyUSB*
```
On some systems (raspberry pi) the user needs to be added to the dialout group to use the serial port. Here we add the user `otto`:
```
$ sudo usermod -a -G dialout otto
```

### Running a Node
To communicate with the Arduino a ROS node needs to be launched on the host system. Assuming a `roscore` is running this can be launched using:
```bash
$ source devel/setup.bash
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

### Send message
Send duty commands
```bash
$ rostopic pub --once /hardware/cmd_joint_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 100, duty_right: 100}'
$ rostopic pub --once /hardware/cmd_joint_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: -100, duty_right: -100}'
$ rostopic pub --once /hardware/cmd_joint_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 0, duty_right: 0}'
```

Control via PID
```bash
$ rostopic pub --once /hardware/motor_pid_gains ottobot_hardware/PidSettings '{kp: 10, ki: 0, kd: 0}'  # Set gains
$ rostopic echo -n 1 /hardware/pid_state # Check gains are set
$ rostopic pub --once /hardware/cmd_joint_state ottobot_hardware/WheelCmd '{mode: 0, angular_velocity_left: 5, angular_velocity_right: 0}'
```

### SerialClient Issue
There is an issue with ROS Melodic and rosserial_arduino ([github issue](https://github.com/ros-drivers/rosserial/pull/414)) where when making a rospy service call to rosserial_arduino the following error occurs:

`"ERROR: service [/topic] responded with an error: service cannot process request: service handler returned None"`

There is no permanent fix as of the time of writing with the temporary solution given as reverting the SerialClient.py file to the kinetic version. A script, `revert_serialclient_to_kinetic.sh`, can be found in the setup folder to complete this action and it is also automated in the `setup_raspi.sh` file.