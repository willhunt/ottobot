---
layout: page
title: Battery State
permalink: /build/battery-state/
---

# Notes on monitoring of battery state using Arduino

## Voltage Divider
<img src="https://render.githubusercontent.com/render/math?math=
V_{out} = V_{in} \cdot \frac{R_2}{R_1 %2B R_2}
" width="auto" height="60">

<img src="https://cdn.sparkfun.com/assets/4/0/3/a/e/511948ffce395f7f47000000.png" alt="Voltage Divider Image" width="auto" height="150">  

*Reference: www.sparkfun.com*

The M0 Pro operates at 3.3V and the batteries used will not go above 10V so the following resistor values are used:

* R1 = 100k ohms
* R2 = 47k ohms

At 10V input the output would be 3.2V.

## Battery Interface
The volatge can be monitored convieniently using the balance pins.