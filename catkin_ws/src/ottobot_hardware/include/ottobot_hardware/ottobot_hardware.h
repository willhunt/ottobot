#ifndef OTTOBOT_HARDWARE_H
#define OTTOBOT_HARDWARE_H

#include "ottobot_hardware_interface.h"
#include <sensor_msgs/JointState>
#include <ottobot_hardware/WheelCmd.h>

class OttobotHardware : OttobotHardwareInterface {
    public:
        OttobotHardware(ros::NodeHandle *n);
        void init_joint_interfaces();
        void read();
        void write();

    private:
        // Subscribe to wheel velocity from microcontroller
        ros::Subscriber wheel_state_subscriber_;
        // Publish wheel velocities to microcontroller
        ros::Publisher wheel_state_publisher_;
        ros::NodeHandle* nh_;
}

#endif // OTTOBOT_HARDWARE_H