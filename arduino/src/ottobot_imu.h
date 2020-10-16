#ifndef OTTOBOT_IMU_H
#define OTTOBOT_IMU_H

#define PUB_INTERVAL_IMU 20  // Delay between updates in milliseconds
#define PUB_INTERVAL_TEMP 5000
#define PUB_INTERVAL_CAL 5000

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <ottobot_sense/ImuCalibration.h>
#include <ottobot_sense/UpdateImuOffsets.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
Rosserial publisher class for publishing IMU data from Adafruit BNO055
*/
class ImuPublisher {
  typedef ottobot_sense::UpdateImuOffsets::Request CalRequest;
  typedef ottobot_sense::UpdateImuOffsets::Response CalResponse;
  
  private:
    ros::NodeHandle *nh_;
    ros::Publisher imu_pub_;
    ros::Publisher temp_pub_;
    ros::Publisher cal_pub_;
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::Temperature temp_msg_;
    ottobot_sense::ImuCalibration cal_msg_;
    long imu_pub_timer_;
    long temp_pub_timer_;
    long cal_pub_timer_;
    Adafruit_BNO055 imu_sensor_;
    void update_offsets_callback();
    ros::ServiceServer<CalRequest, CalResponse> update_cal_server_;
    uint8_t imu_address_;
    uint8_t cal_status_system_;
    uint8_t cal_status_gyro_;
    uint8_t cal_status_accel_;
    uint8_t cal_status_mag_;
    // Store calibration data for use during calibration
    adafruit_bno055_offsets_t calibration_offsets_;

    
  public:
    ImuPublisher(uint8_t imu_address);
    void setup(ros::NodeHandle *nh);
    void publish_imu();
    void publish_temp();
    uint8_t calibration_status();
    void publish_calibration();
};


#endif //OTTOBOT_IMU_H