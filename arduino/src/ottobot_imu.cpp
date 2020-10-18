#include "ottobot_imu.h"

// Node handle reference
ros::NodeHandle* nh_imu_;
// Sensor instance
Adafruit_BNO055 imu_sensor;
// Main data a publisher
long imu_pub_timer;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
// Temperature
long temp_pub_timer;
sensor_msgs::Temperature temp_msg;
ros::Publisher temp_pub("temperature", &temp_msg);
// Calibration
long cal_pub_timer;
ottobot_sense::ImuCalibration cal_msg;
ros::Publisher cal_pub("imu_calibration", &cal_msg);
ros::ServiceServer<CalRequest, CalResponse> update_cal_server("set_imu_offsets", &imu_offsets_callback);
uint8_t cal_status_system, cal_status_gyro, cal_status_accel, cal_status_mag;
// Store calibration data for use during calibration
adafruit_bno055_offsets_t calibration_offsets;

void imu_setup(ros::NodeHandle* nh, uint8_t imu_address) {
    nh_imu_ = nh;
    imu_sensor = Adafruit_BNO055(55, imu_address);
    
    // Publisher - main
    imu_pub_timer = 0;
    imu_msg.header.frame_id = "imu";
    // No measured variance for orientation but put something in
    imu_msg.orientation_covariance[0] = 0.000001;
    imu_msg.orientation_covariance[4] = 0.000001;
    imu_msg.orientation_covariance[8] = 0.000001;
    // Measured values
    imu_msg.linear_acceleration_covariance[0] = 0.00015;
    imu_msg.linear_acceleration_covariance[4] = 0.00015;
    imu_msg.linear_acceleration_covariance[8] = 0.00018;
    // Measured values
    imu_msg.angular_velocity_covariance[0] = 0.011;
    imu_msg.angular_velocity_covariance[4] = 0.021;
    imu_msg.angular_velocity_covariance[8] = 0.006;
    // Advertise
    nh->advertise(imu_pub);

    // Calibration publisher and service
    cal_pub_timer = 0;
    cal_msg.header.frame_id = "imu";
    cal_status_system = 0;
    cal_status_gyro = 0;
    cal_status_accel = 0;
    cal_status_mag = 0;
    nh->advertise(cal_pub);
    nh->advertiseService(update_cal_server);

    // Temperature Publisher
    temp_pub_timer = 0;
    temp_msg.header.frame_id = "imu";
    nh->advertise(temp_pub);

    // Initialize BNO055 Imu
    pinMode(LED_BUILTIN, OUTPUT);
    while(!imu_sensor.begin()) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    delay(1000);
    imu_sensor.setExtCrystalUse(true);

    // Preset calibration offsets for better start data (calibration continues auto)
    calibration_offsets.accel_offset_x = -40;
    calibration_offsets.accel_offset_y = 10;
    calibration_offsets.accel_offset_z = -19;
    calibration_offsets.accel_radius = 1000;
    calibration_offsets.gyro_offset_x = 0;
    calibration_offsets.gyro_offset_y = -2;
    calibration_offsets.gyro_offset_z = -1;
    calibration_offsets.mag_offset_x = -68;
    calibration_offsets.mag_offset_y = 337;
    calibration_offsets.mag_offset_z = 593;
    calibration_offsets.mag_radius = 723;
    imu_sensor.setSensorOffsets(calibration_offsets);
}

/*
Publish internalmeasurement data from BNO055 sensor to ros sensor_msg::Imu specification
*/
void publish_imu() {
    // Check system calibration is complete otherwise don't publish
    // if (millis() > imu_pub_timer && calibration_status() == 3) {
    if (millis() > imu_pub_timer) {
        imu_msg.header.stamp = nh_imu_->now();
        // Orientation
        imu::Quaternion quat = imu_sensor.getQuat();
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        // Angular Velocity
        imu::Vector<3> gyro = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = gyro.x();
        imu_msg.angular_velocity.y = gyro.y();
        imu_msg.angular_velocity.z = gyro.z();

        // Linear Acceleration
        imu::Vector<3> accel = imu_sensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();

        // Publish messages
        imu_pub.publish(&imu_msg);
        
        // Publish at intervals
        imu_pub_timer = millis() + PUB_INTERVAL_IMU;
    }
}

/*
Publish temperature from BNO055 sensor
*/
void publish_imu_temp() {
    if (millis() > temp_pub_timer) {
        int8_t temp = imu_sensor.getTemp();

        temp_msg.temperature = temp;
        temp_msg.header.stamp = nh_imu_->now();

        // Publish messages
        temp_pub.publish(&temp_msg);
        
        // Publish about every 5 seconds
        temp_pub_timer = millis() + PUB_INTERVAL_TEMP;
    }
}

/*
Get calibration status from BNO055 sensor
*/
uint8_t imu_calibration_status() {
    // Get status
    imu_sensor.getCalibration(
        &cal_status_system,
        &cal_status_gyro,
        &cal_status_accel,
        &cal_status_mag
    );
    // Get values
    imu_sensor.getSensorOffsets(calibration_offsets);
    return cal_status_system;
}

/*
Publish temperature from BNO055 sensor
*/
void publish_imu_calibration() {
    if (millis() > cal_pub_timer) {
        imu_calibration_status();
        // Header
        cal_msg.header.stamp = nh_imu_->now();
        // Status
        cal_msg.status.system = cal_status_system;
        cal_msg.status.accelerometer = cal_status_accel;
        cal_msg.status.gyroscope = cal_status_gyro;
        cal_msg.status.magnetometer = cal_status_mag;
        // Offsets
        cal_msg.offsets.accelerometer.x = calibration_offsets.accel_offset_x;
        cal_msg.offsets.accelerometer.y = calibration_offsets.accel_offset_y;
        cal_msg.offsets.accelerometer.z = calibration_offsets.accel_offset_z;
        cal_msg.offsets.accelerometer_radius = calibration_offsets.accel_radius;
        cal_msg.offsets.gyroscope.x = calibration_offsets.gyro_offset_x;
        cal_msg.offsets.gyroscope.y = calibration_offsets.gyro_offset_y;
        cal_msg.offsets.gyroscope.z = calibration_offsets.gyro_offset_z;
        cal_msg.offsets.magnetometer.x = calibration_offsets.mag_offset_x;
        cal_msg.offsets.magnetometer.y = calibration_offsets.mag_offset_y;
        cal_msg.offsets.magnetometer.z = calibration_offsets.mag_offset_z;
        cal_msg.offsets.magnetometer_radius = calibration_offsets.mag_radius;
        // Publish messages
        cal_pub.publish(&cal_msg);
        
        // Publish at intervals
        cal_pub_timer = millis() + PUB_INTERVAL_CAL;
    }
}

/*
Callback for setting calibration offsets
*/
void imu_offsets_callback(const CalRequest& req, CalResponse& res) {
    calibration_offsets.accel_offset_x = req.offsets.accelerometer.x;
    calibration_offsets.accel_offset_y = req.offsets.accelerometer.y;
    calibration_offsets.accel_offset_z = req.offsets.accelerometer.z;
    calibration_offsets.accel_radius = req.offsets.accelerometer_radius;
    calibration_offsets.gyro_offset_x = req.offsets.gyroscope.x;
    calibration_offsets.gyro_offset_y = req.offsets.gyroscope.y;
    calibration_offsets.gyro_offset_z = req.offsets.gyroscope.z;
    calibration_offsets.mag_offset_x = req.offsets.magnetometer.x;
    calibration_offsets.mag_offset_y = req.offsets.magnetometer.y;
    calibration_offsets.mag_offset_z = req.offsets.magnetometer.z;
    calibration_offsets.mag_radius = req.offsets.magnetometer_radius;

    imu_sensor.setSensorOffsets(calibration_offsets);

    res.success = true;
}
