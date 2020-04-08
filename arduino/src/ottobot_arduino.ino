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
#include <sensor_msgs/Imu.h>
#include <Arduino.h>  // Included for use with ROS buildchain
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Address 0x29 for Arduino M0 Pro, usually 0x28 (e.g. Uno)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Set up the ros node and publisher
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);
ros::NodeHandle nh;
long publisher_timer; // Store time until next publish

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
    nh.advertise(pub_imu);
}

void loop(void) 
{
    if (millis() > publisher_timer) {
        /* Get a new sensor event */ 
        sensors_event_t event; 
        bno.getEvent(&event);
        
        // Orientation
        imu::Quaternion quat = bno.getQuat();
        imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y();
        imu_msg.orientation.z = quat.z();
        imu_msg.orientation.w = quat.w();

        // Angular Velocity
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = gyro.x();
        imu_msg.angular_velocity.y = gyro.y();
        imu_msg.angular_velocity.z = gyro.z();

        // Linear Acceleration
        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();

        // Publish messages
        pub_imu.publish(&imu_msg);
        
        // Publish about every 2 seconds
        publisher_timer = millis() + 2000;
    }

    nh.spinOnce();
}