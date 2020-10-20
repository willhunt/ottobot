#include "ottobot_drive.h"
#include <ottobot_hardware/BasePidState.h>

ros::NodeHandle* nh_drive_;

char* joint_names[4] = {
    "front_left_wheel_joint",
    "back_left_wheel_joint",
    "front_right_wheel_joint",
    "back_right_wheel_joint"
};

// Command Subscriber
ros::Subscriber<ottobot_hardware::WheelCmd> wheel_cmd_sub("hardware/cmd_joint_state", &control_cmd_callback);
// PID settings Subscriber
ros::Subscriber<ottobot_hardware::PidSettings> pid_settings_sub("hardware/motor_pid_gains", &pid_settings_callback);
// Joint state publisher
sensor_msgs::JointState joint_state_msg;
unsigned long joint_state_pub_timer = 0;
ros::Publisher joint_state_pub("hardware/joint_states", &joint_state_msg);
// Joint state resetter Subscriber
ros::Subscriber<std_msgs::Bool> joint_reset_sub("hardware/reset_joint_positions", &reset_position_callback);
// Joint state service server
ros::ServiceServer<JointRequest, JointResponse> joint_state_service("/hardware/joint_update", &joint_state_service_callback);
// PID publisher
ottobot_hardware::BasePidState pid_state_msg;
unsigned long pid_state_pub_timer = 0;
ros::Publisher pid_state_pub("hardware/pid_state", &pid_state_msg);
// Mode
char mode = MODE_DUTY;  // 0=PID, 1=Throttle/duty
// PID
double kp = 1.5;
double ki = 15.0;
double kd = 0;
double target_speed_left = 0;  // rad/s
double target_speed_right = 0;  // rad/s
double output_left = 0;  // pwm
double output_right = 0;  // pwm
double position_left = 0;  // rad
double position_right = 0;  // rad
// Assume front and back wheel speeds/positions are the same
double speed_left = 0;  // rad/s
double speed_right = 0;  // rad/s
// Assume front and back wheel speeds/positions are the same
PID pid_left(&speed_left, &output_left, &target_speed_left, kp, ki, kd, DIRECT);
PID pid_right(&speed_right, &output_right, &target_speed_right, kp, ki, kd, DIRECT);
// Wheel encoders
volatile int ticks_left = 0;
volatile int ticks_right = 0;
unsigned long time_last_joint_udpate = 0;  // milliseconds
// Moving average filter
double ma_speed_left_readings[MA_FILTER_WINDOW_SIZE];
double ma_speed_left_sum = 0;
int ma_speed_left_index = 0;
double ma_speed_right_readings[MA_FILTER_WINDOW_SIZE];
double ma_speed_right_sum = 0;
int ma_speed_right_index = 0;

void drive_controller_setup(ros::NodeHandle *nh) {
    nh_drive_ = nh;
    // Advertise publishers
    nh_drive_->advertise(joint_state_pub);
    nh_drive_->advertise(pid_state_pub);
    // Subscribe
    nh_drive_->subscribe(wheel_cmd_sub);
    nh_drive_->subscribe(pid_settings_sub);
    nh_drive_->subscribe(joint_reset_sub);
    // Advertise ServiceServer
    nh_drive_->advertiseService(joint_state_service);

    // Setup wheel PID controllers
    pid_left.SetOutputLimits(-255, 255);
    pid_right.SetOutputLimits(-255, 255);
    // pid_left.SetMode(AUTOMATIC);
    // pid_right.SetMode(AUTOMATIC);

    // Setup motor controller pins
    pinMode(PIN_PWM_LEFT, OUTPUT);
    pinMode(PIN_PWM_RIGHT, OUTPUT);
    pinMode(PIN_DIR_LEFT, OUTPUT);
    pinMode(PIN_DIR_RIGHT, OUTPUT);

    // Attach encoder interrupts
    pinMode(PIN_ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), update_wheel_tick_left_rising, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), update_wheel_tick_left_falling, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), update_wheel_tick_right_rising, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_A), update_wheel_tick_right_falling, FALLING);

    // PID
    pid_left.SetSampleTime(UPDATE_INTERVAL_JOINT_STATE);
    pid_right.SetSampleTime(UPDATE_INTERVAL_JOINT_STATE);

    // Message details - currently done in template function
    joint_state_msg.header.frame_id = "robot_footprint";
    joint_state_msg.name_length = 4;
    joint_state_msg.name = joint_names;
    joint_state_msg.position_length = 4;
    joint_state_msg.velocity_length = 4;
    float joint_efforts[4] = {0, 0, 0, 0};
    joint_state_msg.effort = joint_efforts;  // Populate effort with zeros as unknown.
    // PID state
    pid_state_msg.header.frame_id = "robot_footprint";
    pid_state_msg.output_length = 2;
    pid_state_msg.target_length = 2;
    pid_state_msg.error_length = 2;
    pid_state_msg.pid_mode_length = 2;

    // Setup filter
    for (int i = 0; i < MA_FILTER_WINDOW_SIZE; i++) {
        ma_speed_left_readings[i] = 0;
        ma_speed_right_readings[i] = 0;
    }
}

/*
Calculate wheel speed and position
*/
void update_joint_state() {
    unsigned long time_current = millis();
    // Update at set frequency
    if (time_current - time_last_joint_udpate >= UPDATE_INTERVAL_JOINT_STATE) {
        // Calculate wheel speeds from counted encoder ticks, accumulated via interrupt
        // Left
        double rotations_left = (double)ticks_left / (double)ENC_COUNT_PER_REV;
        double speed_left_new = 1000 * TWO_PI * rotations_left / (double)(time_current - time_last_joint_udpate);  // TWO_PI defined in arduino.h
        // Filter speed
        moving_average_filter(ma_speed_left_sum, ma_speed_left_readings, speed_left, speed_left_new, ma_speed_left_index);
        //Position
        position_left = constrain_angle(position_left + rotations_left * 2 * M_PI);

        // Right
        double rotations_right = (double)ticks_right / (double)ENC_COUNT_PER_REV;
        double speed_right_new = 1000 * TWO_PI * rotations_right / (double)(time_current - time_last_joint_udpate);
        // Filter speed
        moving_average_filter(ma_speed_right_sum, ma_speed_right_readings, speed_right, speed_right_new, ma_speed_right_index);
        // Position
        position_right = constrain_angle(position_right + rotations_right * 2 * M_PI);

        // Reset variables
        time_last_joint_udpate = time_current;
        ticks_left = 0;
        ticks_right = 0;
    }
}

/*
Lag Filter
*/
void lag_filter(double& speed, double& sensor_speed) {
    speed = speed * SPEED_FILTER_VAL + sensor_speed * (1 - SPEED_FILTER_VAL);
}

/*
Moving average filter
*/
void moving_average_filter(double& sum, double* readings, double& speed, double& sensor_speed, int& index) {
// This method does not converge to zero due to doubleing point math but is faster
    // sum -= readings[index];  // Remove the oldest entry from the sum
    // sum += sensor_speed;
// This method is more accurate but includes a short loop
    readings[index] = sensor_speed;
    sum = 0;
    for (int i = 0; i < MA_FILTER_WINDOW_SIZE; i++) {
        sum += readings[i];
    }
    speed = sum / MA_FILTER_WINDOW_SIZE;
    index = (index + 1) % MA_FILTER_WINDOW_SIZE;
    // Round low readings to zero
    // speed = (speed < 0.0000000001) ? 0 : speed;  
}

/*
Increase wheel tick for each interrupt from encoder state change
*/
void update_wheel_tick_left_rising() {
    // Look at other phase to determine direction
    if (digitalRead(PIN_ENCODER_LEFT_B) == LOW) {
        ticks_left ++;  // Forwards
    } else {
        ticks_left --;  // Backwards
    }
}
void update_wheel_tick_left_falling() {
    // Look at other phase to determine direction
    if (digitalRead(PIN_ENCODER_LEFT_B) == HIGH) {
        ticks_left ++;  // Forwards
    } else {
        ticks_left --;  // Backwards
    }
}
void update_wheel_tick_right_rising() {
    if (digitalRead(PIN_ENCODER_RIGHT_B) == HIGH) {
        ticks_right ++;  // Forwards
    } else {
        ticks_right --;  // Backwards
    }
}
void update_wheel_tick_right_falling() {
    if (digitalRead(PIN_ENCODER_RIGHT_B) == LOW) {
        ticks_right ++;  // Forwards
    } else {
        ticks_right --;  // Backwards
    }
}

/*
Update PID
*/
void drive_controller_update() {
    // Update sensor reading here or at different frequency? Do it here for now
    update_joint_state();
    // If voltage is low do not run the motors
    if (low_voltage_cut_off) {
        output_left = 0;
        output_right = 0;
    } else if (mode == MODE_PID) {
        // Update pid's
        pid_left.Compute();
        pid_right.Compute();
    }
    drive_motors(output_left, output_right);
}

/*
Set output to motors based upon direction and magnitude
Outputs are in range -255 to 255
*/
void drive_motors(double output_left, double output_right) {
    // output_left = map(abs(output_left), 0, 100, 0, 255);
    if (output_left < 0) {
        // Limit min & max magnitude
        output_left = (output_left < -LIMIT_DUTY_MAX) ? -LIMIT_DUTY_MAX : output_left;
        output_left = (output_left > -LIMIT_DUTY_MIN) ? 0 : output_left;
        analogWrite(PIN_PWM_LEFT, -output_left);
        digitalWrite(PIN_DIR_LEFT, HIGH);  // Backwards
    } else {
        output_left = (output_left > LIMIT_DUTY_MAX) ? LIMIT_DUTY_MAX : output_left;
        output_left = (output_left < LIMIT_DUTY_MIN) ? 0 : output_left;
        analogWrite(PIN_PWM_LEFT, output_left);
        digitalWrite(PIN_DIR_LEFT, LOW);  // Forwards
    }
    
    // output_right = map(abs(output_right), 0, 100, 0, 255);
    if (output_right < 0) {
        output_right = (output_right < -LIMIT_DUTY_MAX) ? -LIMIT_DUTY_MAX : output_right;
        output_right = (output_right > -LIMIT_DUTY_MIN) ? 0 : output_right;
        analogWrite(PIN_PWM_RIGHT,  -output_right);
        digitalWrite(PIN_DIR_RIGHT, LOW);  // Backwards
    } else {
        output_right = (output_right > LIMIT_DUTY_MAX) ? LIMIT_DUTY_MAX : output_right;
        output_right = (output_right < LIMIT_DUTY_MIN) ? 0 : output_right;
        analogWrite(PIN_PWM_RIGHT,  output_right);
        digitalWrite(PIN_DIR_RIGHT, HIGH);  // Forwards
    }    
}

/*
Publish wheel speed and position
*/
void publish_joint_state() {
    if (millis() > joint_state_pub_timer) {
        // populate_joint_msg(joint_state_msg);

        joint_state_msg.header.stamp = nh_drive_->now();
        float joint_positions[4] = {position_left, position_left, position_right, position_right};
        joint_state_msg.position = joint_positions;
        float joint_speeds[4] = {speed_left, speed_left, speed_right, speed_right};
        joint_state_msg.velocity = joint_speeds;

        joint_state_pub.publish(&joint_state_msg);
        // Publish at interval
        joint_state_pub_timer = millis() + PUB_INTERVAL_JOINT_STATE;
    }
}

/*
    Populate joint values of message type sensor_msgs/JointState
    Can be from message or service response types
*/
void populate_joint_msg(sensor_msgs::JointState& input) {
    input.header.stamp = nh_drive_->now();
    input.header.frame_id = "robot_footprint";
    input.name_length = 4;
    input.name = joint_names;
    input.position_length = 4;
    input.velocity_length = 4;
    input.effort_length = 4;

    float joint_positions[4] = {position_left, position_left, position_right, position_right};
    input.position = joint_positions;
    float joint_speeds[4] = {speed_left, speed_left, speed_right, speed_right};
    input.velocity = joint_speeds;
    float joint_efforts[4] = {0, 0, 0, 0};
    input.effort = joint_efforts;
}

/*
Return wheel speed and position as service server
*/
void joint_state_service_callback(const JointRequest& request, JointResponse& response) {
    populate_joint_msg(response.state);
}

/*
Callback for wheel speed commands
*/
void control_cmd_callback(const ottobot_hardware::WheelCmd& cmd_msg) {
    if (cmd_msg.mode == MODE_PID) {
        target_speed_left = cmd_msg.angular_velocity_left;
        target_speed_right = cmd_msg.angular_velocity_right;
        // If commands are zero, turn PID off to ensure stationary
        if (target_speed_left == 0) {
            pid_left.SetMode(MANUAL);
            output_left = 0;
        } else if (pid_left.GetMode() == MANUAL) {  // Turn individual pid on if off
            pid_left.SetMode(AUTOMATIC);
        }
        if (target_speed_right == 0) {
            pid_right.SetMode(MANUAL);
            output_right = 0;
        } else if (pid_right.GetMode() == MANUAL) {  // Turn individual pid on if off
            pid_right.SetMode(AUTOMATIC);
        }
        mode = MODE_PID;
    } else if (cmd_msg.mode == MODE_DUTY) {
        if (mode == MODE_PID) {  // Turn pid off
            pid_left.SetMode(MANUAL);
            pid_right.SetMode(MANUAL);
        }
        mode = MODE_DUTY;
        output_left = cmd_msg.duty_left;
        output_right = cmd_msg.duty_right;
        target_speed_left = 0;
        target_speed_right = 0;
    }
}

/*
Publish pid state
*/
void publish_pid_state() {
    if (millis() > pid_state_pub_timer) {
        pid_state_msg.header.stamp = nh_drive_->now();
        // Assume front and back wheel speeds/positions are the same
        float errors[2] = {target_speed_left - speed_left, target_speed_right - speed_right};
        pid_state_msg.error = errors;
        float target_speeds[2] = {target_speed_left, target_speed_right};
        pid_state_msg.target = target_speeds;
        float outputs[2] = {output_left, output_right};
        pid_state_msg.output = outputs;
        char* pid_modes[2] = {
            (pid_left.GetMode() == MANUAL) ? (char*)"Manual" : (char*)"Auto",
            (pid_right.GetMode() == MANUAL) ? (char*)"Manual" : (char*)"Auto"
        };
        pid_state_msg.pid_mode = pid_modes;
        pid_state_msg.cmd_mode = (mode == 0) ? "PID" : "Duty";
        pid_state_msg.p_term = pid_left.GetKp();
        pid_state_msg.i_term = pid_left.GetKi();
        pid_state_msg.d_term = pid_left.GetKd();

        pid_state_pub.publish(&pid_state_msg);
        // Publish about every 0.1 seconds
        pid_state_pub_timer = millis() + PUB_INTERVAL_PID_STATE;
    }
}

/*
Update PID gains
*/
void set_drive_gains(double kp, double ki, double kd) {
    pid_left.SetTunings(kp, ki, kd);
    pid_right.SetTunings(kp, ki, kd);
}

/*
Constrain angle to 0 -> 2pi
*/
double constrain_angle(double theta) {
    if (LIMIT_POSITION_TO_2PI){
        theta = fmod(theta, TWO_PI);
        if (theta < 0)
            theta += TWO_PI;
    }    
    return theta;
}

/*
Callback for PID gain update
*/
void pid_settings_callback(const ottobot_hardware::PidSettings& settings_msg) {
    set_drive_gains(settings_msg.kp, settings_msg.ki, settings_msg.kd);
}

/*
Reset joint position
*/
void reset_position_callback(const std_msgs::Bool& msg) {
    if (msg.data) {
        position_left = 0;
        position_right = 0;
    }
}