---
layout: page
title: Kinect
description: Xbox 360 kinect setup.
---

# USB Conversion
The Xbox 360 kinect operates at:
* Voltage: 12V 
* Maximum power: 12W (1A)

To use this with the battery running 7.4V the voltage needs to be stepped up. I used a "Mini Voltage COnverter Step-Up Module" from [magic-stone-us](https://www.ebay.co.uk/str/magic-stone-us?_trksid=p2047675.l2563) on eBay.  

<img src="{{ site.baseurl}}/assets/notes_voltage_regulator.jpg" alt="Voltage Divider Image" width="auto" height="150"> 

# ROS library
Suggested package to use:
https://wiki.ros.org/openni_camera