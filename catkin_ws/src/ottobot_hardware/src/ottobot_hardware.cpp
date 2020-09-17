#include "ottobot_hardware_interface.h"

OttobotHardware::OttobotHardware(ros::NodeHandle *nh) :
    nh_(nh)
{
    // Setup joint interfaces
    init_joint_interfaces();
    // Subscribe to wheel state from microcontroller
    wheel_state_subscriber_ = n->subscribe("/robot_wheel_state", 10, 
            &OttobotHW::wheel_state_callback, this);
    // Publish wheel state from microcontroller
     wheel_state_publisher_ = n->advertise<sensor_msgs::JointState>("cmd_wheel_state", 1);

}

OttobotHardware::init_joint_interfaces() {
    // Get joint names
    nh_->getParam("/ottobot/hardware_interface/joints", joint_names_);

    for (int i = 0; i < 4; i++) {
        // Register joint interface for getting state
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names_[i], &position_[i], &velocity_[i], &effort_[i]
        );
        joint_state_interface_.registerHandle(joint_state_handle);

        // Register joint interface for setting state
        hardware_interface::JointHandle joint_handle_(
            joint_state_interface.getHandle(joint_names_[i]), &cmd_[i]
        )
        joint_vel_interface_.registerHandle(joint_handle_);

        // Do joint limits here later **************
    }
    registerInterface(&joint_state_interface__);
    registerInterface(&joint_vel_interface_);
}

OttobotHardware::wheel_state_callback(const sensor_msgs::JointState state_msg) {
    // Go through joints and store state
    for (int obj_joint_index = 0; obj_joint_index < 4; obj_joint_index++) {
        // find index of joint name in message
        int msg_joint_index;
        for (int j = 0; j < state_msg.name.size(); j++) {
            if (state_msg.name[j] == joint_names_[obj_joint_index]) {
                msg_joint_index = j;
                break;
            }
        }
        // Populate joint values
        position_[obj_joint_index] = state_msg.position[msg_joint_index];
        velocity_[obj_joint_index] = state_msg.velocity[msg_joint_index];
        effort_[obj_joint_index] = state_msg.effort[msg_joint_index];
    }
}

OttobotHardware::write() {
    // Publish command for microcontroller
    cmd_msg = ottobot_hardware::WheelCmd;
    cms_msg.angular_velocity_left = cmd_[0];
    cms_msg.angular_velocity_right = cmd_[2];
    wheel_state_publisher_.publish(cmd_msg);
}