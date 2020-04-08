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
// #define USE_M0PRO_NATIVE

#include <ros.h>
#include <Arduino.h>  // Included for use with ROS buildchain
#include "ottobot_imu.h"

// Set up the ros node and publisher
ros::NodeHandle nh;
// Address 0x29 for Arduino M0 Pro, usually 0x28 (e.g. Uno)
ImuPublisher imu_publisher(0x29);

void setup(void)
{
    // ROS node setup
    nh.initNode();
    // IMU setup
    imu_publisher.setup(&nh);
}

void loop(void) 
{
    // imu_publisher.pub_bno(bno);
    imu_publisher.publish_imu();

    nh.spinOnce();
}