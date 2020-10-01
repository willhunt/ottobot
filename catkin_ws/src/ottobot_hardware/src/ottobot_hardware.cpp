#include "ottobot_hardware/ottobot_hardware.h"
#include <ottobot_hardware/WheelCmd.h>
#include <ottobot_hardware/JointUpdate.h>
#include <ros/console.h>

OttobotHardware::OttobotHardware(ros::NodeHandle* nh) :
    nh_(nh)
{
    // Setup joint interfaces
    init_joint_interfaces();
    // Subscribe to wheel state from microcontroller
    // wheel_state_subscriber_ = nh_->subscribe("/hardware/joint_states", 10, 
    //         &OttobotHardware::wheel_state_callback, this);
    // Publish wheel state for microcontroller
    wheel_state_publisher_ = nh_->advertise<ottobot_hardware::WheelCmd>("/hardware/cmd_joint_state", 1);
    // Setup service client for sending requests to arduino
    joint_service_client_ = nh_->serviceClient<ottobot_hardware::JointUpdate>("/hardware/joint_update");
}

void OttobotHardware::init_joint_interfaces() {
    // Get joint names
    nh_->getParam("/ottobot/hardware_interface/joints", joint_names_);

    for (int i = 0; i < N_JOINTS; i++) {
        // Register joint interface for getting state
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names_[i], &position_[i], &velocity_[i], &effort_[i]
        );
        joint_state_interface_.registerHandle(joint_state_handle);

        // Register joint interface for setting state
        hardware_interface::JointHandle joint_handle_(
            joint_state_interface_.getHandle(joint_names_[i]),
            &cmd_[i]
        );
        joint_vel_interface_.registerHandle(joint_handle_);

        // Do joint limits here later **************
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_vel_interface_);
}

void OttobotHardware::wheel_state_callback(const sensor_msgs::JointState joint_state_msg) {
    set_joint_values<sensor_msgs::JointState>(joint_state_msg);
}

void OttobotHardware::write() {
    // Publish command for microcontroller
    ottobot_hardware::WheelCmd cmd_msg;
    cmd_msg.angular_velocity_left = cmd_[0];
    cmd_msg.angular_velocity_right = cmd_[2];
    wheel_state_publisher_.publish(cmd_msg);
}

/**
 * Request update from arduino on joint states
**/
void OttobotHardware::read() {
    ottobot_hardware::JointUpdate update_srv;
    if (joint_service_client_.call(update_srv)) {
        set_joint_values<ottobot_hardware::JointUpdate::Response>(update_srv.response);
    } else {
        ROS_ERROR("Failed to call joint update service request");
    } 
}
