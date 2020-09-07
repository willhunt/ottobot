#include "ottobot_drive.h"


DriveController::DriveController(
    int pin_pwm_left, int pin_dir_left, int pin_encoder_left,
    int pin_pwm_right, int pin_dir_right, int pin_encoder_left
) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor exectutes
    odom_pub_("odom", &odom_msg_)
{
    odom_pub_timer_ = 0;

    pin_pwm_left_ = pin_pwm_left;
    pin_dir_left_ = pin_dir_left;
    pin_pwm_right_ = pin_pwm_right;
    pin_dir_right_ = pin_dir_right;
    pin_encoder_left_ = pin_encoder_left;
    pin_encoder_right_ = pin_encoder_right;

    target_speed_left_ = 0;
    target_speed_right_ = 0;
    actual_speed_left_ = 0;
    actual_speed_right_ = 0;
    output_left_ = 0;
    output_right_ = 0;

    kp_ = 1.0;
    ki_ = 0.001;
    kd_ = 0;

    pid_wheels_left_ = PID(&actual_speed_left_, &actual_speed_left_, &target_speed_left_, kp_, ki_, kd_, DIRECT);
    pid_wheels_right_ = PID(&actual_speed_right_, &actual_speed_right_, &target_speed_right_, kp_, ki_, kd_, DIRECT);

    wheel_speed_left_ = 0;
    wheel_speed_right_ = 0;
    wheel_ticks_left_ = 0;
    wheel_ticks_right_ = 0;
    time_last_speed_udpate_ = 0;
}

void DriveController::setup(ros::NodeHandle *nh) {
    nh_ = nh;
    // Advertise rosserial publisher
    nh->advertise(odom_pub_);

    // Turn the PID on
    pid_wheels_left_.SetMode(AUTOMATIC);
    pid_wheels_right_.SetMode(AUTOMATIC);

    // Attach encoder interrupts
    pinMode(pin_encoder_left_, INPUT_PULLUP);
    pinMode(pin_encoder_right_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_left_), update_wheel_tick_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_encoder_right_), update_wheel_tick_right, CHANGE);
}

/*
Publish internalmeasurement data from BNO055 sensor to ros sensor_msg::Imu specification
*/
void DriveController::publish_odom() {
    if (millis() > odom_pub_timer_) {
        
        odom_msgs_.blah = blah
        // Publish messages
        odom_pub_.publish(&odom_msg_);
        
        // Publish about every 2 seconds
        odom_pub_timer_ = millis() + 2000;
    }
}

/*
Calculate wheel speeds
*/
void DriveController::update_wheel_speeds() {
    long time_current = millis();
    // Calculate wheel speeds from counted encoder ticks, accumlated via interrupt
    double rotations_left = double(wheel_ticks_left_ / ENC_COUNT_PER_REV);
    wheel_speed_left_ = TWO_PI * rotations / double(time_current - time_last_speed_udpate_);  // TWO_PI defined in arduino.h
    double rotations_right = double(wheel_ticks_right_ / ENC_COUNT_PER_REV);
    wheel_speed_left_ = TWO_PI * rotations_right / double(time_current - time_last_speed_udpate_);

    // Reset variables
    time_last_speed_udpate_ = time_current;
    wheel_ticks_left_ = 0;
    wheel_ticks_right_ = 0;
}

/*
Increase wheel tick for each interrupt from encoder state change
*/
void DriveController::update_wheel_tick_left() {
    wheel_ticks_left_ += 1;
}
void DriveController::update_wheel_tick_right() {
    wheel_ticks_right_ += 1;
}

/*
Update PID
*/
void DriveController::drive_update() {
    // Update sensor reading

    pid_wheels_left_.Compute();
    pid_wheels_right_.Compute();

    analogWrite(pin_pwm_left_, output_left_);
    analogWrite(pin_pwm_right_, output_right_);
}


