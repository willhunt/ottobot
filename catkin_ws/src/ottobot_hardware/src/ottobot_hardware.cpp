#include "ottobot_hardware/ottobot_hardware.h"
#include <ottobot_hardware/WheelCmd.h>
#include <ottobot_hardware/JointUpdate.h>
#include <ros/console.h>
#include <ros/service.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#define USE_SUB_FOR_UPDATE false  // Use subscriber instead of service to update joint values

OttobotHardwareInterface::OttobotHardwareInterface(ros::NodeHandle* nh) :
    nh_(nh)
{
    // Setup joint interfaces
    init_joint_interfaces();
    if (USE_SUB_FOR_UPDATE) {
        // Subscribe to wheel state from microcontroller
        wheel_state_subscriber_ = nh_->subscribe("/hardware/joint_states", 10, 
                &OttobotHardwareInterface::wheel_state_callback, this);
    } else {
        // Wait for joint state service server to start
        ros::service::waitForService("/hardware/joint_update", 2000);
        // Setup service client for sending requests to arduino
        joint_service_client_ = nh_->serviceClient<ottobot_hardware::JointUpdate>("/hardware/joint_update");
    }
    // Publish wheel state for microcontroller
    wheel_state_publisher_ = nh_->advertise<ottobot_hardware::WheelCmd>("/hardware/cmd_joint_state", 1);

}

void OttobotHardwareInterface::init_joint_interfaces() {
    // Get joint names
    nh_->getParam("/ottobot/hardware_interface/joints", joint_names_);

    for (int i = 0; i < N_JOINTS; i++) {
        // Register joint interface for getting state
        hardware_interface::JointStateHandle joint_state_handle(
            joint_names_[i], &position_[i], &velocity_[i], &effort_[i]
        );
        joint_state_interface_.registerHandle(joint_state_handle);

        // Register joint interface for setting state
        hardware_interface::JointHandle joint_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &cmd_[i]
        );
        joint_vel_interface_.registerHandle(joint_handle);

        // Joint limits
        joint_limits_interface::JointLimits joint_limits;
        // Get limits from parameter server (joint_limits_params.yaml)
        const bool rosparam_limits_ok = getJointLimits("wheel_joint", *nh_, joint_limits);
        if (rosparam_limits_ok) {
            joint_limits_interface::VelocityJointSaturationHandle joint_limits_handle(
                joint_handle,
                joint_limits
            );
            joint_limits_interface_.registerHandle(joint_limits_handle);
        } else {
            ROS_ERROR("Could not load joint limits from ROS parameter server");
        }
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_vel_interface_);
}

void OttobotHardwareInterface::wheel_state_callback(const sensor_msgs::JointState joint_state_msg) {
    set_joint_values<sensor_msgs::JointState>(joint_state_msg);
}

void OttobotHardwareInterface::write(ros::Duration elapsed_time) {
    // Enforce joint limits for all registered handles
    joint_limits_interface_.enforceLimits(elapsed_time);

    // Publish command for microcontroller
    ottobot_hardware::WheelCmd cmd_msg;
    cmd_msg.mode = 0;
    cmd_msg.angular_velocity_left = cmd_[0];
    cmd_msg.angular_velocity_right = cmd_[2];
    wheel_state_publisher_.publish(cmd_msg);
}

/**
 * Request update from arduino on joint states
 * Not working properly, possibly due to open issue: https://github.com/ros-drivers/rosserial/issues/408
 * Use subscriber to update instead
**/
void OttobotHardwareInterface::read() {
    if (USE_SUB_FOR_UPDATE) {
        return;
    } else {
        ottobot_hardware::JointUpdate update_srv;
        if (joint_service_client_.call(update_srv)) {
            set_joint_values<ottobot_hardware::JointUpdate::Response>(update_srv.response);
        } else {
            ROS_ERROR("Failed to call joint update service request");
        } 
    }
}
