#include "ottobot_imu.h"


ImuPublisher::ImuPublisher() :
    // Initialising member variables before the body of the constructor executes
    imu_pub_("imu", &imu_msg_)
{
    imu_pub_timer_ = 0;
}

void ImuPublisher::setup(ros::NodeHandle *nh) {
    nh->advertise(imu_pub_);
}

void ImuPublisher::pub_bno(Adafruit_BNO055 &bno) {
    if (millis() > imu_pub_timer_) {
        // sensor_msgs::Imu imu_msg;
        // Orientation
        imu::Quaternion quat = bno.getQuat();
        imu_msg_.orientation.x = quat.x();
        imu_msg_.orientation.y = quat.y();
        imu_msg_.orientation.z = quat.z();
        imu_msg_.orientation.w = quat.w();

        // Angular Velocity
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg_.angular_velocity.x = gyro.x();
        imu_msg_.angular_velocity.y = gyro.y();
        imu_msg_.angular_velocity.z = gyro.z();

        // Linear Acceleration
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg_.linear_acceleration.x = accel.x();
        imu_msg_.linear_acceleration.y = accel.y();
        imu_msg_.linear_acceleration.z = accel.z();

        // Publish messages
        imu_pub_.publish(&imu_msg_);
        
        // Publish about every 2 seconds
        imu_pub_timer_ = millis() + 2000;
    }
}
