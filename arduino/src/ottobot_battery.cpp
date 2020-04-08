#include "ottobot_battery.h"


BatteryPublisher::BatteryPublisher(int battery_voltage_pin, double max_voltage, int adc_resolution_bits = 10) :
    // Initialising member variables before the body of the constructor executes
    //     From testing Arduino requires publisher to be defined before constructor exectutes
    state_pub_("battery_state", &state_msg_)
{
    state_pub_timer_ = 0;
    battery_voltage_pin_ = battery_voltage_pin;
    max_voltage_ = max_voltage;
    adc_resolution_bits_ = adc_resolution_bits;
}

void BatteryPublisher::setup(ros::NodeHandle *nh) {
    nh_ = nh;
    // Advertise rosserial imu publisher
    nh->advertise(state_pub_);
    // Maximise ADC resolution for M0 Pro
    analogReadResolution(adc_resolution_bits_);
}

/*
Publish battery state as measured by voltage divider
*/
void BatteryPublisher::publish_state() {
    if (millis() > state_pub_timer_) {
        int sensorValue = analogRead(battery_voltage_pin_);
        double voltage = map(sensorValue, 0, pow(2, adc_resolution_bits_), 0, max_voltage_);

        state_msg_.voltage = voltage;
        state_msg_.header.stamp = nh_->now();

        // Publish messages
        state_pub_.publish(&state_msg_);
        
        // Publish about every 2 seconds
        state_pub_timer_ = millis() + 2000;
    }
}
