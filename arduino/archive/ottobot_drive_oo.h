#ifndef OTTOBOT_DRIVE_H
#define OTTOBOT_DRIVE_H

#include <Arduino.h>
#include <ros.h>
#include <PID_v1.h>
// #include <PID_AutoTune_v0.h>
// #include <geometry_msgs/Twist.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
#include <ottobot_hardware/WheelCmd.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/PidState.h>

#define ENC_COUNT_PER_REV 1200

/* 
Rosserial drive class for controlling wheel speed and publishing odometry & transform data
*/
class DriveController {
  
  private:
    static DriveController* instance; // Required for ISR's

    ros::NodeHandle *nh_;

    ros::Publisher joint_state_pub_;
    unsigned long joint_state_pub_timer_;
    sensor_msgs::JointState joint_state_msg_;

    ros::Publisher pid_state_pub_;
    unsigned long pid_state_pub_timer_;
    control_msgs::PidState pid_state_msg_;

    ros::Subscriber<ottobot_hardware::WheelCmd, DriveController> wheel_cmd_sub_;
    void wheel_speed_cmd_callback(const ottobot_hardware::WheelCmd&);

    static void DriveController::update_wheel_tick_left_isr();
    static void DriveController::update_wheel_tick_right_isr();
  
    double wheel_position_left_;  // rad
    double wheel_position_right_;  // rad
    double wheel_speed_left_;  // rad/s
    double wheel_speed_right_;  // rad/s
    static volatile double wheel_ticks_left_;
    static volatile double wheel_ticks_right_;
    unsigned long time_last_speed_udpate_;  // milliseconds

    PID pid_wheels_left_;
    PID pid_wheels_right_;
    int pin_pwm_left_;
    int pin_dir_left_;
    int* pins_encoder_left_;
    int pin_pwm_right_ ;
    int pin_dir_right_;
    int* pins_encoder_right_;

    double target_speed_left_;  // rad/s
    double target_speed_right_;  // rad/s
    double output_left_;  // pwm
    double output_right_;  // pwm

    // double dia_wheel_;  // Wheel diameter [m]
    char* joint_names_[4] = {
      "front_left_wheel_joint",
      "back_left_wheel_joint",
      "front_right_wheel_joint",
      "back_right_wheel_joint"
    };
  
  public:
    DriveController(
      int pin_pwm_left,
      int pin_dir_left,
      int pins_encoder_left[2],
      int pin_pwm_right,
      int pin_dir_right,
      int pins_encoder_right[2],
      int kp, int ki, int kd
    );
    void setup(ros::NodeHandle *nh);

    void publish_joint_state();
    void publish_pid_state();

    void update_joint_state();
    void drive_update();
    void update_wheel_tick_left();
    void update_wheel_tick_right();

    void update();

    void set_gains(double kp, double ki, double kd);
};


#endif //OTTOBOT_DRIVE_H