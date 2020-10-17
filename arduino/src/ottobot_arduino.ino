/*
Author: Willaim Hunt
Date: April 2020
Project: Ottobot

Description: Arduino files for Ottobot mobile robot using ROS serial communication
Notes: Requires modification to rosserial package to work with M0 Pro Arduino board
*/
// #define ARDUINO 1000  // Make sure to use Arduino.h not WProgram.h
#define USE_USBCON
// Select USB port on M0 Pro:
#define USE_M0PRO_PROGRAMMING
// Define USE_M0PRO_NATIVE
#define BATTERY_VOLTAGE_PIN A0
// Define number of bits to use for analog-digital conversion
//     UNO:     10
//     M0 Pro:  12
#define ADC_RESOLUTION_BITS 12

#include <ros.h>
#include <Arduino.h>  // Included for use with ROS buildchain
#include "ottobot_imu.h"
#include "ottobot_battery.h"
#include "ottobot_drive.h"

// Set up the ros node and publisher
ros::NodeHandle nh;
// Setup battery monitor (pin, Vmax)
BatteryPublisher battery_publisher(BATTERY_VOLTAGE_PIN, 10.352, ADC_RESOLUTION_BITS);

void setup(void)
{
    // ROS node setup
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    // IMU setup - Address 0x29 for Arduino M0 Pro, usually 0x28 (e.g. Uno)
    imu_setup(&nh, 0x29);
    // Battery monitor setup
    battery_publisher.setup(&nh);
    // Motor controller setup
    drive_controller_setup(&nh);
}

void loop(void) 
{
    // Publish IMU main data
    publish_imu();
    publish_imu_calibration();
    // Publish IMU temperature
    publish_imu_temp();
    // Publish battery state (voltage only atm)
    battery_publisher.publish_state();
    // Update PID and output motor commands
    drive_controller_update();
    // Publish motor control messages
    publish_joint_state();
    publish_pid_state();

    nh.spinOnce();
}