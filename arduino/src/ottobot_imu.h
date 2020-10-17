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

typedef ottobot_sense::UpdateImuOffsets::Request CalRequest;
typedef ottobot_sense::UpdateImuOffsets::Response CalResponse;

void imu_setup(ros::NodeHandle *nh, uint8_t imu_address);
void publish_imu();
void publish_imu_temp();
void publish_imu_calibration();
void imu_offsets_callback(const CalRequest& req, CalResponse& res);
uint8_t imu_calibration_status();

#endif //OTTOBOT_IMU_H