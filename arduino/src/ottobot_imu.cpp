#include "ottobot_imu.h"


ImuPublisher::ImuPublisher(uint8_t imu_address) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor exectutes
    imu_pub_("imu", &imu_msg_),
    temp_pub_("temperature", &temp_msg_),
    imu_sensor_(55, imu_address)
{
    imu_pub_timer_ = 0;
    imu_address_ = imu_address;
    pinMode(LED_BUILTIN, OUTPUT);
    // Default message parameters
    imu_msg_.header.frame_id = "imu";
}

void ImuPublisher::setup(ros::NodeHandle *nh) {
    nh_ = nh;
    // Advertise rosserial imu publisher
    nh->advertise(imu_pub_);
    nh->advertise(temp_pub_);

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

/*
Publish internalmeasurement data from BNO055 sensor to ros sensor_msg::Imu specification
*/
void ImuPublisher::publish_imu() {
    if (millis() > imu_pub_timer_) {
        imu_msg_.header.stamp = nh_->now();
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

/*
Publish temperature from BNO055 sensor
*/
void ImuPublisher::publish_temp() {
    if (millis() > temp_pub_timer_) {
        int8_t temp = imu_sensor_.getTemp();

        temp_msg_.temperature = temp;
        temp_msg_.header.stamp = nh_->now();

        // Publish messages
        temp_pub_.publish(&temp_msg_);
        
        // Publish about every 5 seconds
        temp_pub_timer_ = millis() + 5000;
    }
}

