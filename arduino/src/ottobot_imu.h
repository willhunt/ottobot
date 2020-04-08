#ifndef OTTOBOT_IMU_H
#define OTTOBOT_IMU_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class ImuPublisher {
  
  private:
    ros::Publisher imu_pub_;
    sensor_msgs::Imu imu_msg_;
    long imu_pub_timer_;
    ros::NodeHandle nh_;
    
  public:
    ImuPublisher();
    void setup(ros::NodeHandle *nh);
    void pub_bno(Adafruit_BNO055 &bno);

};

#endif //OTTOBOT_IMU_H