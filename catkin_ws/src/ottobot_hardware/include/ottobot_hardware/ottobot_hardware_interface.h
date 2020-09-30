#ifndef OTTOBOT_HW_INTERFACE_H
#define OTTOBOT_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class OttobotHardwareInterface : public hardware_interface::RobotHW {
public:
    // OttobotHardwareInterface();

protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface joint_vel_interface_;
    // joint_limits_interface::VelocityJointSaturationInterface joint_vel_saturation_interface_;
    // joint_limits_interface::VelocityJointSoftLimitsInterface joint_vel_limits_interface_;
    // 4 joints
    std::vector<double> cmd_ = std::vector<double>(4);
    std::vector<double> position_ = std::vector<double>(4);
    std::vector<double> velocity_ = std::vector<double>(4);
    std::vector<double> effort_ = std::vector<double>(4);
    std::vector<std::string> joint_names_ = std::vector<std::string>(4);
};

#endif // OTTOBOT_HW_INTERFACE_H