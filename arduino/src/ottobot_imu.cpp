#include "ottobot_imu.h"


ImuPublisher::ImuPublisher(uint8_t imu_address) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor exectutes
    imu_pub_("imu", &imu_msg_),
    imu_sensor_(55, imu_address)
{
    imu_pub_timer_ = 0;
    imu_address_ = imu_address;
    pinMode(LED_BUILTIN, OUTPUT);
}

void ImuPublisher::setup(ros::NodeHandle *nh) {
    // Advertise rosserial imu publisher
    nh->advertise(imu_pub_);

    // Initialize BNO055 Imu
    while(!imu_sensor_.begin()) {
        // If code is stuck here...
        //  - Maybe ADR pin is not pulled high for M0 Pro?
        //  - Maybe woring is wrong?
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    delay(1000);
    imu_sensor_.setExtCrystalUse(true);
}

void ImuPublisher::publish_imu() {
    if (millis() > imu_pub_timer_) {
        // sensor_msgs::Imu imu_msg;
        // Orientation
        imu::Quaternion quat = imu_sensor_.getQuat();
        imu_msg_.orientation.x = quat.x();
        imu_msg_.orientation.y = quat.y();
        imu_msg_.orientation.z = quat.z();
        imu_msg_.orientation.w = quat.w();

        // Angular Velocity
        imu::Vector<3> gyro = imu_sensor_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg_.angular_velocity.x = gyro.x();
        imu_msg_.angular_velocity.y = gyro.y();
        imu_msg_.angular_velocity.z = gyro.z();

        // Linear Acceleration
        imu::Vector<3> accel = imu_sensor_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg_.linear_acceleration.x = accel.x();
        imu_msg_.linear_acceleration.y = accel.y();
        imu_msg_.linear_acceleration.z = accel.z();

        // Publish messages
        imu_pub_.publish(&imu_msg_);
        
        // Publish about every 2 seconds
        imu_pub_timer_ = millis() + 2000;
    }
}


