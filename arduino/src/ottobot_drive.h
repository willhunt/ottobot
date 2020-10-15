#ifndef OTTOBOT_DRIVE_H
#define OTTOBOT_DRIVE_H

#include <Arduino.h>
#include <ros.h>
#include <PID_v1.h>
// #include <PID_AutoTune_v0.h>
#include <ottobot_hardware/WheelCmd.h>
#include <ottobot_hardware/PidSettings.h>
#include <ottobot_hardware/JointUpdate.h>
#include <sensor_msgs/JointState.h>

#define ENC_COUNT_PER_REV 300  //1200
#define PIN_PWM_LEFT 5  // PWM1
#define PIN_PWM_RIGHT 7  // PWM2
#define PIN_DIR_LEFT 4  // DIR1
#define PIN_DIR_RIGHT 6  // DIR2
#define PIN_ENCODER_LEFT_A 11  // Yellow M1
#define PIN_ENCODER_LEFT_B 12  // White M1
#define PIN_ENCODER_RIGHT_A 9  // Yellow M2
#define PIN_ENCODER_RIGHT_B 10  // White M2

#define PUB_INTERVAL_JOINT_STATE 20  // [ms]
#define PUB_INTERVAL_PID_STATE 20  // [ms]
#define UPDATE_INTERVAL_JOINT_STATE 20  // [ms]
#define SPEED_FILTER_VAL 0.8
#define MA_FILTER_WINDOW_SIZE 10

#define LIMIT_DUTY_MIN 0
#define LIMIT_DUTY_MAX 255

#define MODE_PID 0
#define MODE_DUTY 1

void drive_controller_setup(ros::NodeHandle* nh);
void control_cmd_callback(const ottobot_hardware::WheelCmd& cmd_msg);
void update_tick_left();
void update_tick_right();
void publish_joint_state();
void publish_pid_state();
void update_joint_state();
void update_wheel_tick_left_rising();
void update_wheel_tick_left_falling();
void update_wheel_tick_right_rising();
void update_wheel_tick_right_falling();
void drive_controller_update();
void set_drive_gains(double kp, double ki, double kd);
void pid_settings_callback(const ottobot_hardware::PidSettings& settings_msg);
double constrain_angle(double theta);
void drive_motors(double output_left, double output_right);
typedef ottobot_hardware::JointUpdate::Request JointRequest;
typedef ottobot_hardware::JointUpdate::Response JointResponse;
void joint_state_service_callback(const JointRequest& request, JointResponse& response);
void moving_average_filter(double& sum, double* readings, double& speed, double& sensor_speed, int& index);
void lag_filter(double& speed, double& sensor_speed);

extern bool low_voltage_cut_off;

#endif //OTTOBOT_DRIVE_H