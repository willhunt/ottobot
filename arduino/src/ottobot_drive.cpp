#include "ottobot_drive.h"

ros::NodeHandle *nh_;

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
// PID publisher
control_msgs::PidState pid_state_msg;
unsigned long pid_state_pub_timer = 0;
ros::Publisher pid_state_pub("hardware/pid_left_state", &pid_state_msg);
// Mode
char mode = 1;  // 0=PID, 1=Throttle/duty
// PID
double kp = 5;
double ki = 25;
double kd = 0;
double target_speed_left = 0;  // rad/s
double target_speed_right = 0;  // rad/s
double output_left = 0;  // pwm
double output_right = 0;  // pwm
double position_left = 0;  // rad
double position_right = 0;  // rad
// Assume front and back wheel speeds/positions are the same
// float positions[4] = {&position_left, &position_left, &position_right, &position_right};
double speed_left = 0;  // rad/s
double speed_right = 0;  // rad/s
// Assume front and back wheel speeds/positions are the same
// float speeds[4] = {&speed_left, &speed_left, &speed_right, &speed_right};
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
    nh_ = nh;
    // Advertise publishers
    nh_->advertise(joint_state_pub);
    nh_->advertise(pid_state_pub);
    // Subscribe to cmd topic
    nh_->subscribe(wheel_cmd_sub);
    // Subscribe to pid settings topic
    nh_->subscribe(pid_settings_sub);

    // Setup wheel PID controllers
    pid_left.SetOutputLimits(-255, 255);
    pid_right.SetOutputLimits(-255, 255);
    pid_left.SetMode(AUTOMATIC);
    pid_right.SetMode(AUTOMATIC);

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

    // Message details
    joint_state_msg.header.frame_id = "robot_footprint";
    joint_state_msg.name_length = 4;
    joint_state_msg.name = joint_names;
    // joint_state_msg.position = positions;
    joint_state_msg.position_length = 4;
    joint_state_msg.velocity_length = 4;
    // joint_state_msg.velocity = speeds;
    pid_state_msg.header.frame_id = "robot_footprint";

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
        // Lag filter
        // speed_left = speed_left * SPEED_FILTER_VAL + speed_left_new * (1 - SPEED_FILTER_VAL);
        // Moving average filter
        // ma_speed_left_sum -= ma_speed_left_readings[ma_speed_left_index];  // Remove the oldest entry from the sum
        // ma_speed_left_sum += speed_left_new;  // Add new value to sum
        ma_speed_left_readings[ma_speed_left_index] = speed_left_new;  // Replace with new value in store
        ma_speed_left_index = (ma_speed_left_index + 1) % MA_FILTER_WINDOW_SIZE;  // Increment index
        ma_speed_left_sum = 0;
        for (int i = 0; i < MA_FILTER_WINDOW_SIZE; i++) {
            ma_speed_left_sum += ma_speed_left_readings[i];
        }
        speed_left = ma_speed_left_sum / MA_FILTER_WINDOW_SIZE;
        // speed_left = (speed_left < 0.0000000001) ? 0 : speed_left;  // Round low readings to zero
        //Position
        position_left = constrain_angle(position_left + rotations_left);

        // Right
        double rotations_right = (double)ticks_right / (double)ENC_COUNT_PER_REV;
        double speed_right_new = 1000 * TWO_PI * rotations_right / (double)(time_current - time_last_joint_udpate);
        // Filter speed
        // Lag filter
        // speed_right = speed_right * SPEED_FILTER_VAL + speed_right_new * (1 - SPEED_FILTER_VAL);
        // Moving average filter
        // ma_speed_right_sum -= ma_speed_right_readings[ma_speed_right_index];  // Remove the oldest entry from the sum
        // ma_speed_right_sum += speed_right_new;
        ma_speed_right_sum = 0;
        for (int i = 0; i < MA_FILTER_WINDOW_SIZE; i++) {
            ma_speed_right_sum += ma_speed_right_readings[i];
        }
        ma_speed_right_readings[ma_speed_right_index] = speed_right_new;
        ma_speed_right_index = (ma_speed_right_index + 1) % MA_FILTER_WINDOW_SIZE;
        speed_right = ma_speed_right_sum / MA_FILTER_WINDOW_SIZE;
        // speed_right = (speed_right < 0.0000000001) ? 0 : speed_right;  // Round low readings to zero
        // Position
        position_right = constrain_angle(position_right + rotations_right);
        // Reset variables
        time_last_joint_udpate = time_current;
        ticks_left = 0;
        ticks_right = 0;
    }
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
    } else if (mode == 0) {  // PID mode
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
        joint_state_msg.header.stamp = nh_->now();

        float joint_positions[4] = {position_left, position_left, position_right, position_right};
        joint_state_msg.position = joint_positions;

        float joint_speeds[4] = {speed_left, speed_left, speed_right, speed_right};
        // float joint_speeds[4] = {0.0, 0.0, 0.0, 0.0};  // This looked fine.
        joint_state_msg.velocity = joint_speeds;

        // Populate effort with zeros as unknown.
        float joint_efforts[4] = {0, 0, 0, 0};
        joint_state_msg.effort = joint_efforts;

        joint_state_pub.publish(&joint_state_msg);
        // Publish about every 0.5 seconds
        joint_state_pub_timer = millis() + PUB_INTERVAL_JOINT_STATE;
    }
}

/*
Callback for wheel speed commands
*/
void control_cmd_callback(const ottobot_hardware::WheelCmd& cmd_msg) {
    if (cmd_msg.mode == 0) {
        if (mode != 0) {  // Turn pid on if off
            pid_left.SetMode(AUTOMATIC);
            pid_right.SetMode(AUTOMATIC);
            mode = 0;
        }
        target_speed_left = cmd_msg.angular_velocity_left;
        target_speed_right = cmd_msg.angular_velocity_right;
    } else if (cmd_msg.mode == 1) {
        if (mode == 0) {  // Turn pid off
            pid_left.SetMode(MANUAL);
            pid_right.SetMode(MANUAL);
        }
        mode = 1;
        output_left = cmd_msg.duty_left;
        output_right = cmd_msg.duty_right;
    }
}

/*
Publish pid state
*/
void publish_pid_state() {
    if (millis() > pid_state_pub_timer) {
        // Assume front and back wheel speeds/positions are the same
        pid_state_msg.error = target_speed_left - speed_left;
        pid_state_msg.output = output_left;
        pid_state_msg.p_term = pid_left.GetKp();
        pid_state_msg.i_term = pid_left.GetKi();
        pid_state_msg.d_term = pid_left.GetKd();
        pid_state_msg.header.stamp = nh_->now();
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
    theta = fmod(theta, TWO_PI);
    if (theta < 0)
        theta += TWO_PI;
    return theta;
}

/*
Callback for PID gain update
*/
void pid_settings_callback(const ottobot_hardware::PidSettings& settings_msg) {
    set_drive_gains(settings_msg.kp, settings_msg.ki, settings_msg.kd);
}