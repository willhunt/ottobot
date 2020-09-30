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

## ROS Interfacing
To communicate with the Arduino a ROS node needs to be launched on the host system. Assuming a `roscore` is running this can be launched using:
```bash
$ source devel/setup.bash
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

### Send message
Send duty commands
```bash
$ rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 100, duty_right: 100}'
$ rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: -100, duty_right: -100}'
$ rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 0, duty_right: 0}'
```

Control via PID
```bash
$ rostopic pub --once /motor_pid_gains ottobot_hardware/PidSettings '{kp: 10, ki: 0, kd: 0}'  # Set gains
$ rostopic echo -n 1 /pid_left_state # Check gains are set
$ rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 0, angular_velocity_left: 5, angular_velocity_right: 0}'
```