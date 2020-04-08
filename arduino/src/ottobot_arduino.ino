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

// Address 0x29 for Arduino M0 Pro, usually 0x28 (e.g. Uno)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
// Set up the ros node and publisher
ros::NodeHandle nh;
ImuPublisher imu_publisher;

void setup(void)
{
    // BN0555 Setup
    Serial.begin(57600);
    Serial.println("Checking BNO055"); Serial.println("");
    /* Initialise the sensor */
    if(!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    } else {
        Serial.println("BN0555 detected");
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    // ROS node setup
    nh.initNode();
    imu_publisher.setup(&nh);
}

void loop(void) 
{
    imu_publisher.pub_bno(bno);

    nh.spinOnce();
}