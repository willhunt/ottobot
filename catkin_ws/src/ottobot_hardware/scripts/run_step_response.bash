#!/bin/sh

# source ../../../devel/setup.bash

rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 150, duty_right: 0}'
sleep 8
rostopic pub --once /cmd_wheel_state ottobot_hardware/WheelCmd '{mode: 1, duty_left: 0, duty_right: 0}'