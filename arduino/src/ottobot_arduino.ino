/*
Author: Willaim Hunt
Date: April 2020
Project: Ottobot

Description: Ardunio files for Ottobot mobile robot using ROS serial communication
Notes: Requires modification to rosserial package to work with M0 Pro Arduino board
*/

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

// Set up the ros node and publisher
ros::NodeHandle nh;
// Address 0x29 for Arduino M0 Pro, usually 0x28 (e.g. Uno)
ImuPublisher imu_publisher(0x29);
// Setup battery monitor (pin, Vmax)
BatteryPublisher battery_publisher(BATTERY_VOLTAGE_PIN, 10, ADC_RESOLUTION_BITS);

void setup(void)
{
    // ROS node setup
    nh.initNode();
    // IMU setup
    imu_publisher.setup(&nh);
    // Battery monitor setup
    battery_publisher.setup(&nh);
}

void loop(void) 
{
    // Publish IMU main data
    imu_publisher.publish_imu();
    // Publish IMU temperature
    imu_publisher.publish_temp();
    // Publish battery state (voltage only atm)
    battery_publisher.publish_state();

    nh.spinOnce();
}