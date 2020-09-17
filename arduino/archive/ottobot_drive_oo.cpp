#include "ottobot_drive.h"


DriveController::DriveController(
    int pin_pwm_left, int pin_dir_left, int pins_encoder_left[2],
    int pin_pwm_right, int pin_dir_right, int pins_encoder_right[2],
    int kp, int ki, int kd
) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor exectutes
    joint_state_pub_("joint_state", &joint_state_msg_),
    pid_state_pub_("pid_left_state", &pid_state_msg_)
    // wheel_cmd_sub_("/cmd_wheel_state", &DriveController::wheel_speed_cmd_callback, this)
{
    

    pin_pwm_left_ = pin_pwm_left;
    pin_dir_left_ = pin_dir_left;
    pin_pwm_right_ = pin_pwm_right;
    pin_dir_right_ = pin_dir_right;
    pins_encoder_left_ = pins_encoder_left;
    pins_encoder_right_ = pins_encoder_right;

    target_speed_left_ = 0;
    target_speed_right_ = 0;
    output_left_ = 0;
    output_right_ = 0;

    wheel_speed_left_ = 0;
    wheel_speed_right_ = 0;
    wheel_position_left_ = 0;
    wheel_position_right_ = 0;

    pid_wheels_left_ = PID(&wheel_speed_left_, &output_left_, &target_speed_left_, kp, ki, kd, DIRECT);
    pid_wheels_right_ = PID(&wheel_speed_right_, &output_right_, &target_speed_right_, kp, ki, kd, DIRECT);

    wheel_ticks_left_ = 0;
    wheel_ticks_right_ = 0;
    time_last_speed_udpate_ = 0;

    // Joint state publisher
    joint_state_pub_timer_ = 0;
    joint_state_msg_.header.frame_id = "robot_footprint";
    joint_state_msg_.name = joint_names_;
    // PID state publisher
    pid_state_pub_timer_ = 0;
    pid_state_msg_.header.frame_id = "robot_footprint";
}

void DriveController::setup(ros::NodeHandle *nh) {
    nh_ = nh;
    // Advertise rosserial publisher
    nh_->advertise(joint_state_pub_);
    // Subscribe to cmd topic
    // wheel_cmd_sub_ = nh_->subscribe("/cmd_wheel_state", 10, &DriveController::wheel_speed_cmd_callback, this);
    wheel_cmd_sub_ = ros::Subscriber("/cmd_wheel_state", &DriveController::wheel_speed_cmd_callback, this);
    nh_->subscribe(wheel_cmd_sub_);

    // Turn the PID on
    pid_wheels_left_.SetMode(AUTOMATIC);
    pid_wheels_right_.SetMode(AUTOMATIC);

    instance = this;
    // Attach encoder interrupts
    for (int i = 0; i < 2; i++) {
        pinMode(pins_encoder_left_[i], INPUT_PULLUP);
        pinMode(pins_encoder_right_[i], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pins_encoder_left_[i]), update_wheel_tick_left, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pins_encoder_right_[i]), update_wheel_tick_right, CHANGE);
    }
}

/*
Calculate wheel speeds
*/
void DriveController::update_joint_state() {
    unsigned long time_current = millis();
    // Calculate wheel speeds from counted encoder ticks, accumlated via interrupt
    double rotations_left = (double)wheel_ticks_left_ / (double)ENC_COUNT_PER_REV;
    wheel_speed_left_ = TWO_PI * rotations_left / (double)(time_current - time_last_speed_udpate_);  // TWO_PI defined in arduino.h
    double rotations_right = (double)wheel_ticks_right_ / (double)ENC_COUNT_PER_REV;
    wheel_speed_left_ = TWO_PI * rotations_right / (double)(time_current - time_last_speed_udpate_);

    // Reset variables
    time_last_speed_udpate_ = time_current;
    wheel_ticks_left_ = 0;
    wheel_ticks_right_ = 0;
}

/*
Increase wheel tick for each interrupt from encoder state change
*/
void DriveController::update_wheel_tick_left_isr() {
    instance->update_wheel_tick_left();
}
void DriveController::update_wheel_tick_right_isr() {
    instance->update_wheel_tick_right();
}
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
    // Update sensor reading here or at different frequency?
    // Do it here for now
    update_joint_state();

    pid_wheels_left_.Compute();
    pid_wheels_right_.Compute();

    analogWrite(pin_pwm_left_, output_left_);
    analogWrite(pin_pwm_right_, output_right_);
}


/*
Publish wheel speed and position
*/
void DriveController::publish_joint_state() {
    if (millis() > joint_state_pub_timer_) {
        // Assume front and back wheel speeds/positions are the same
        joint_state_msg_.position = {
            wheel_position_left_, wheel_position_left_,
            wheel_position_right_, wheel_position_right_
        };
        joint_state_msg_.velocity = {
            wheel_speed_left_, wheel_speed_left_,
            wheel_speed_right_, wheel_speed_right_
        };
        joint_state_msg_.header.stamp = nh_->now();
        joint_state_pub_.publish(&joint_state_msg_);

        // Publish about every 0.5 seconds
        joint_state_pub_timer_ = millis() + 500;
    }
}

/*
Callback for wheel speed commands
*/
void DriveController::wheel_speed_cmd_callback(const ottobot_hardware::WheelCmd& cmd_msg) {
    target_speed_left_ = cmd_msg.angular_velocity_left;
    target_speed_right_ = cmd_msg.angular_velocity_right;
}

/*
Publish pid state
*/
void DriveController::publish_pid_state() {
    if (millis() > pid_state_pub_timer_) {
        // Assume front and back wheel speeds/positions are the same
        pid_state_msg_.error = target_speed_left_ - wheel_speed_left_;
        pid_state_msg_.output = output_left_;
        pid_state_msg_.p_term = pid_wheels_left_.GetKp();
        pid_state_msg_.i_term = pid_wheels_left_.GetKi();
        pid_state_msg_.d_term = pid_wheels_left_.GetKd();
        pid_state_msg_.header.stamp = nh_->now();
        pid_state_pub_.publish(&pid_state_msg_);

        // Publish about every 0.1 seconds
        pid_state_pub_timer_ = millis() + 100;
    }
}

/*
Update PID gains
*/
void DriveController::set_gains(double kp, double ki, double kd) {
    pid_wheels_left_.SetTunings(kp, ki, kd);
    pid_wheels_right_.SetTunings(kp, ki, kd);
}