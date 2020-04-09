---
layout: page
title: Arduino
permalink: /build/arduino/
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
The rosserial Arduino library does not support the M0 Pro so a few changes have to be made. Install the latest version from [github](https://github.com/ros-drivers/rosserial) via the instruction on the ROS [wiki](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). Then replace the file '/rol_lib/ArduinoHardware.h' with [ArduinoHardware.h](arduino/reference/ArduinoHardware.h). In summary of the changes, some cases have been added to force the serial definitions based upon use of the M0 Pro:

```c
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
```c
#define USE_USBCON
// Use this for Native USB port
#define USE_M0PRO_NATIVE
// Use this for Programming USB port
// #define USE_M0PRO_PROGRAMMING
```
