#ifndef OTTOBOT_DRIVE_H
#define OTTOBOT_DRIVE_H

#include <Arduino.h>
#include <ros.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define ENC_COUNT_PER_REV 16

/* 
Rosserial drive class for controlling wheel speed and publishing odometry & transform data
*/
class DriveController {
  
  private:
    ros::NodeHandle *nh_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    nav_msgs::Odometry odom_msg_;
    sensor_msgs::Temperature temp_msg_;

    long odom_pub_timer_;

    double wheel_speed_left_;  // rpm
    double wheel_speed_right_;  // rpm
    static volatile double wheel_ticks_left_;
    static volatile double wheel_ticks_right_;
    long time_last_speed_udpate_;  // milliseconds

    PID pid_wheels_left_;
    PID pid_wheels_right_;
    int pin_pwm_left_;
    int pin_dir_left_;
    int pin_pwm_right_ ;
    int pin_dir_right_;

    double target_speed_left_;  // rpm
    double target_speed_right_;  // rpm
    double actual_speed_left_;  // rpm
    double actual_speed_right_;  // rpm
    double output_left_;  // pwm
    double output_right_;  // pwm

    double kp_;
    double ki_;
    double kd_;
    
  public:
    DriveController();
    void setup(ros::NodeHandle *nh);
    void publish_odom();
    void broadcast_odom();
    void twist_callback();
    void drive_update();
    void update_wheel_tick_left();
    void update_wheel_tick_right();
};


#endif //OTTOBOT_DRIVE_H