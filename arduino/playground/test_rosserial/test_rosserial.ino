/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
#define USE_USBCON
// #define USE_M0PRO_PROGRAMMING
#define USE_M0PRO_NATIVE


#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
    Serial.begin(57600);
    Serial.println("Starting up...");
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(chatter);
}

void loop()
{
    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
    delay(1000);
}