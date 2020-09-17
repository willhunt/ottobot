#ifndef OTTOBOT_DRIVE_H
#define OTTOBOT_DRIVE_H

#include <Arduino.h>
#include <ros.h>
#include <PID_v1.h>
// #include <PID_AutoTune_v0.h>
#include <ottobot_hardware/WheelCmd.h>
#include <ottobot_hardware/PidSettings.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/PidState.h>

#define ENC_COUNT_PER_REV 300  //1200
#define PIN_PWM_LEFT 5  // PWM1
#define PIN_PWM_RIGHT 7  // PWM2
#define PIN_DIR_LEFT 4  // DIR1
#define PIN_DIR_RIGHT 6  // DIR2
#define PIN_ENCODER_LEFT_A 9  // Yellow M1
#define PIN_ENCODER_LEFT_B 10  // White M1
#define PIN_ENCODER_RIGHT_A 11  // Yellow M2
#define PIN_ENCODER_RIGHT_B 12  // White M2

void drive_controller_setup(ros::NodeHandle* nh);
void control_cmd_callback(const ottobot_hardware::WheelCmd& cmd_msg);
void update_tick_left();
void update_tick_right();
void publish_joint_state();
void publish_pid_state();
void update_joint_state();
void update_wheel_tick_left();
void update_wheel_tick_right();
void drive_controller_update();
void set_drive_gains(double kp, double ki, double kd);
void pid_settings_callback(const ottobot_hardware::PidSettings& settings_msg);
double constrain_angle(double theta);
void drive_motors(double output_left, double output_right);

extern bool low_voltage_cut_off;

#endif //OTTOBOT_DRIVE_H