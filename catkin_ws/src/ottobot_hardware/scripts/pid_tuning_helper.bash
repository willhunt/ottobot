#!/bin/sh

source ../../../devel/setup.bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200 &

rqt_plot /pid_left_state/error /pid_left_state/output /joint_state/velocity[0]
