#include "ottobot_hardware/ottobot_hardware.h"
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>

/**
    Control loop for ottobot
    Source: http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
*/
void control_loop(OttobotHardwareInterface& hw_interface, controller_manager::ControllerManager& cm, ros::Time& last_time) {
    ros::Time this_time = ros::Time::now();
    ros::Duration elapsed_time = this_time - last_time;
    // Process control loop
    // hw_interface.read();
    cm.update(ros::Time::now(), elapsed_time);
    hw_interface.write(elapsed_time);

    last_time = this_time;
}

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "ottobot_hardware_node");
    // Create a ROS NodeHandle object
    ros::NodeHandle nh;
    // Instantiate Hardware Interface
    OttobotHardwareInterface hw_interface = OttobotHardwareInterface(&nh);
    // Add to controller manager
    controller_manager::ControllerManager cm(&hw_interface, nh);

    // Create controller queue and spinner
    ros::CallbackQueue control_queue;
    ros::AsyncSpinner control_spinner(1, &control_queue); // 1 thread
    // NB timers are not realtime safe
    ros::Time last_time = ros::Time::now();
    ros::TimerOptions control_timer_options(
        ros::Duration(0.1),
        boost::bind(control_loop, boost::ref(hw_interface), boost::ref(cm), boost::ref(last_time)),
        &control_queue);
    ros::Timer control_timer = nh.createTimer(control_timer_options);
    // Start spinner for controller update
    control_spinner.start();
    // Still use ros spin for pub/sub etc.
    ros::spin();
}