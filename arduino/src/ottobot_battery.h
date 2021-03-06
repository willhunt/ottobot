#ifndef OTTOBOT_BATTERY_H
#define OTTOBOT_BATTERY_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/BatteryState.h>

#define CUT_OFF_VOLTAGE 6.8  // 3.4V minimum per cell (2 cells)

/* 
Rosserial publisher class for publishing battery state
*/
class BatteryPublisher {
  
  private:
    ros::NodeHandle *nh_;
    ros::Publisher state_pub_;
    sensor_msgs::BatteryState state_msg_;
    long state_pub_timer_;
    int battery_voltage_pin_;
    double max_voltage_;
    int adc_resolution_bits_;
    
  public:
    BatteryPublisher(int battery_voltage_pin, double max_voltage, int adc_resolution_bits);
    void setup(ros::NodeHandle *nh);
    void publish_state();
    double read_state();
};


#endif //OTTOBOT_BATTERY_H