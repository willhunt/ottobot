#ifndef OTTOBOT_IMU_H
#define OTTOBOT_IMU_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
Rosserial publisher class for publishing IMU data from Adafruit BNO055
*/
class ImuPublisher {
  
  private:
    ros::NodeHandle *nh_;
    ros::Publisher imu_pub_;
    ros::Publisher temp_pub_;
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::Temperature temp_msg_;
    long imu_pub_timer_;
    long temp_pub_timer_;
    Adafruit_BNO055 imu_sensor_;
    uint8_t imu_address_;
    
  public:
    ImuPublisher(uint8_t imu_address);
    void setup(ros::NodeHandle *nh);
    void publish_imu();
    void publish_temp();
};


#endif //OTTOBOT_IMU_H