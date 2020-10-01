#!/bin/sh

# Remove existing libraries
rm -rf $HOME/Arduino/libraries/ros_lib
# Recreate
rosrun rosserial_arduino make_libraries.py $HOME/Arduino/libraries
# Copy modified ArduinoHardware.h file
cp ../reference/ArduinoHardware.h $HOME/Arduino/libraries/ros_lib
