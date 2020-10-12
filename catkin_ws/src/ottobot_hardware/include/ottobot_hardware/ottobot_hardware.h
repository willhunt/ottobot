#ifndef OTTOBOT_HARDWARE_H
#define OTTOBOT_HARDWARE_H

#define N_JOINTS 4

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>

class OttobotHardwareInterface : public hardware_interface::RobotHW {
    public:
        OttobotHardwareInterface(ros::NodeHandle* n);
        void read();
        void write(ros::Duration elapsed_time);

    private:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface joint_vel_interface_;
        joint_limits_interface::VelocityJointSaturationInterface joint_limits_interface_;
        std::vector<double> cmd_ = std::vector<double>(N_JOINTS);
        std::vector<double> position_ = std::vector<double>(N_JOINTS);
        std::vector<double> velocity_ = std::vector<double>(N_JOINTS);
        std::vector<double> effort_ = std::vector<double>(N_JOINTS);
        std::vector<std::string> joint_names_ = std::vector<std::string>(N_JOINTS);
        // Subscribe to wheel velocity from microcontroller
        ros::Subscriber wheel_state_subscriber_;
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
template<typename T> void OttobotHardwareInterface::set_joint_values(T input) {
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