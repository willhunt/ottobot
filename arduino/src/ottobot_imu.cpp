#include "ottobot_imu.h"

ImuPublisher::ImuPublisher(uint8_t imu_address) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor executes
    imu_pub_("imu", &imu_msg_),
    temp_pub_("temperature", &temp_msg_),
    cal_pub_("imu_calibration", &cal_msg_),
    update_cal_server_("set_imu_offsets", &ImuPublisher::update_offsets_callback, this),
    imu_sensor_(55, imu_address)
{
    imu_pub_timer_ = 0;
    imu_address_ = imu_address;
    pinMode(LED_BUILTIN, OUTPUT);
    // Default message parameters
    imu_msg_.header.frame_id = "imu";

    // No measured variance for orientation but put something in
    imu_msg_.orientation_covariance[0] = 0.00001;
    imu_msg_.orientation_covariance[4] = 0.00001;
    imu_msg_.orientation_covariance[8] = 0.00001;
    // Measured values
    imu_msg_.linear_acceleration_covariance[0] = 0.00007;
    imu_msg_.linear_acceleration_covariance[4] = 0.00008;
    imu_msg_.linear_acceleration_covariance[8] = 0.0052;
    // Measured values
    imu_msg_.angular_velocity_covariance[0] = 0.0101;
    imu_msg_.angular_velocity_covariance[4] = 0.0196;
    imu_msg_.angular_velocity_covariance[8] = 0.0052;

    // Calibration defaults
    cal_pub_timer_ = 0;
    cal_msg_.header.frame_id = "imu";
    cal_status_system_ = 0;
    cal_status_gyro_ = 0;
    cal_status_accel_ = 0;
    cal_status_mag_ = 0;

    // Temperature defaults
    temp_pub_timer_ = 0;
    temp_msg_.header.frame_id = "imu";
}

void ImuPublisher::setup(ros::NodeHandle *nh) {
    nh_ = nh;
    // Advertise rosserial imu publisher
    nh->advertise(imu_pub_);
    nh->advertise(temp_pub_);
    nh->advertise(cal_pub_);
    nh->advertiseService(update_cal_server_);

    // Initialize BNO055 Imu
    while(!imu_sensor_.begin()) {
        // If code is stuck here...
        //  - Maybe ADR pin is not pulled high for M0 Pro?
        //  - Maybe wiring is wrong?
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
    // Check system calibration is complete otherwise don't publish
    if (millis() > imu_pub_timer_ && calibration_status() ==3) {
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
        
        // Publish at intervals
        imu_pub_timer_ = millis() + PUB_INTERVAL_IMU;
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
        temp_pub_timer_ = millis() + PUB_INTERVAL_TEMP;
    }
}

/*
Get calibration status from BNO055 sensor
*/
uint8_t ImuPublisher::calibration_status() {
    // Get status
    imu_sensor_.getCalibration(
        &cal_status_system_,
        &cal_status_gyro_,
        &cal_status_accel_,
        &cal_status_mag_
    );
    // Get values
    imu_sensor_.getSensorOffsets(calibration_offsets_);
    return cal_status_system_;
}

/*
Publish temperature from BNO055 sensor
*/
void ImuPublisher::publish_calibration() {
    if (millis() > cal_pub_timer_) {
        calibration_status();
        // Header
        cal_msg_.header.stamp = nh_->now();
        // Status
        cal_msg_.status.system = cal_status_system_;
        cal_msg_.status.accelerometer = cal_status_accel_;
        cal_msg_.status.gyroscope = cal_status_gyro_;
        cal_msg_.status.magnetometer = cal_status_mag_;
        // Offsets
        cal_msg_.offsets.accelerometer.x = calibration_offsets_.accel_offset_x;
        cal_msg_.offsets.accelerometer.y = calibration_offsets_.accel_offset_y;
        cal_msg_.offsets.accelerometer.z = calibration_offsets_.accel_offset_z;
        cal_msg_.offsets.accelerometer_radius = calibration_offsets_.accel_radius;
        cal_msg_.offsets.gyroscope.x = calibration_offsets_.gyro_offset_x;
        cal_msg_.offsets.gyroscope.y = calibration_offsets_.gyro_offset_y;
        cal_msg_.offsets.gyroscope.z = calibration_offsets_.gyro_offset_z;
        cal_msg_.offsets.magnetometer.x = calibration_offsets_.mag_offset_x;
        cal_msg_.offsets.magnetometer.y = calibration_offsets_.mag_offset_y;
        cal_msg_.offsets.magnetometer.z = calibration_offsets_.mag_offset_z;
        cal_msg_.offsets.magnetometer_radius = calibration_offsets_.mag_radius;
        // Publish messages
        cal_pub_.publish(&cal_msg_);
        
        // Publish at intervals
        cal_pub_timer_ = millis() + PUB_INTERVAL_CAL;
    }
}

/*
Callback for setting calibration offsets
*/
void ImuPublisher::update_offsets_callback(const CalRequest & req, CalResponse & res) {
    calibration_offsets_.accel_offset_x = req.offsets.accelerometer.x;
    calibration_offsets_.accel_offset_y = req.offsets.accelerometer.y;
    calibration_offsets_.accel_offset_z = req.offsets.accelerometer.z;
    calibration_offsets_.accel_radius = req.offsets.accelerometer_radius;
    calibration_offsets_.gyro_offset_x = req.offsets.gyroscope.x;
    calibration_offsets_.gyro_offset_y = req.offsets.gyroscope.y;
    calibration_offsets_.gyro_offset_z = req.offsets.gyroscope.z;
    calibration_offsets_.mag_offset_x = req.offsets.magnetometer.x;
    calibration_offsets_.mag_offset_y = req.offsets.magnetometer.y;
    calibration_offsets_.mag_offset_z = req.offsets.magnetometer.z;
    calibration_offsets_.mag_radius = req.offsets.magnetometer_radius;
    res.success = true;
}
