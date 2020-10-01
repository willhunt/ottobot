#ifndef OTTOBOT_HARDWARE_H
#define OTTOBOT_HARDWARE_H

#define N_JOINTS 4

#include <ros/ros.h>
#include "ottobot_hardware/ottobot_hardware_interface.h"
#include <sensor_msgs/JointState.h>

class OttobotHardware : public OttobotHardwareInterface {
    public:
        OttobotHardware(ros::NodeHandle* n);
        void read();
        void write();

    private:
        // Subscribe to wheel velocity from microcontroller
        // ros::Subscriber wheel_state_subscriber_;
        // Publish wheel velocities to microcontroller
        ros::Publisher wheel_state_publisher_;
        // Client capable of requesting state service from arduino
        ros::ServiceClient joint_service_client_;
        ros::NodeHandle* nh_;
        void wheel_state_callback(const sensor_msgs::JointState state_msg);
        void init_joint_interfaces();
        template<typename T> void set_joint_values(T input);
};

/**
 * Set joint values from either msg or srv
 * Definition in header due to being template method
**/
template<typename T> void OttobotHardware::set_joint_values(T input) {
    // Go through message/service and update joints provided
    for (int i_input = 0; i_input < input.name.size(); i_input++) {
        // Loop through joints to find each one
        int found_joint_index = -1;
        for (int i_obj = 0; i_obj < N_JOINTS; i_obj++) {
            if (input.name[i_input].compare(joint_names_[i_obj]) == 0) {
                found_joint_index = i_obj;
                break;
            }
        }
        if (found_joint_index == -1) {
            // ROS_ERROR("Joint not found: %s", input.name[i_input].c_str());
            ROS_ERROR("Joint not found");
        } else {
            // Populate joint values, checking that items are not empty (e.g. effort)
            if (input.position.size() >= i_input + 1)
                position_[found_joint_index] = input.position[i_input];
            if (input.velocity.size() >= i_input + 1)
                velocity_[found_joint_index] = input.velocity[i_input];
            if (input.effort.size() >= i_input + 1)
                effort_[found_joint_index] = input.effort[i_input];
        }
    }
}

#endif // OTTOBOT_HARDWARE_H