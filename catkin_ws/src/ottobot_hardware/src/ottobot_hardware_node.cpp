#include "ottobot_hardware.h"

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "ottobot_base");
    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    // Instantiate Hardware Interface
    OttobotHardware hw_interface = OttobotHardware(&n);

    // Publish the interface
    while (hw_interface.getNumSubscribers() < 1)
    {
        if (!ros::ok()) {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the hw interface");
        sleep(1);
    }

    // hw_interface.init();

    ros::spin();

}