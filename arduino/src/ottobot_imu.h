#ifndef OTTOBOT_IMU_H
#define OTTOBOT_IMU_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
Rosserial publisher class for publishing IMU data from Adafruit BNO055
*/
class ImuPublisher {
  
  private:
    ros::Publisher imu_pub_;
    sensor_msgs::Imu imu_msg_;
    long imu_pub_timer_;
    ros::NodeHandle nh_;
    Adafruit_BNO055 imu_sensor_;
    uint8_t imu_address_;
    
  public:
    ImuPublisher(uint8_t imu_address);
    void setup(ros::NodeHandle *nh);
    // void publish_imu(Adafruit_BNO055 &bno);
    void publish_imu();
};


#endif //OTTOBOT_IMU_H