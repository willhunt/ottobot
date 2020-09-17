---
images_divider: [
    {src: "voltage_divider.png", caption: "Voltage divider"},
]
images_wiring: [
    {src: "voltage_divider_fritzing.png", caption: "Voltage divider wiring"},
]
images_connector: [
    {src: "detail_battery_state_01.jpg", caption: "Cell balance monitoring connection"},
]
---
# Battery Connection
Battery connection and voltage monitoring with Arduino and voltage divider.

Key components of power draw:

Component         |  Max Power Draw [W] | 
:-------------    |:-----------------: 
4 Motors          | 81.6          
Raspberry Pi      | 12.8
Kinect            | 12.0
**Total**         | 106.4

At 7.4V a maximum of 12.8A could be drawn from the battery meaning a [cable size](https://www.solar-wind.co.uk/info/dc-cable-wire-sizing-tool-low-voltage-drop-calculator) of 14AWG is required.

# Battery State Monitoring
Notes on monitoring the battery volatge as an indication of charge level using the Arduino via a voltage divider.

## Voltage Divider
The Arduino M0 Pro operates at 3.3V so to read the battery voltage it must be scaled down using a [voltage divider](https://learn.sparkfun.com/tutorials/voltage-dividers/all). The batteries used will not go above 10V so the following resistor values are used:

* R1 = 100k ohms
* R2 = 47k ohms

At 10V input the output would be 3.2V.

$$V_{out} = V_{in} \cdot \frac{R_2}{R_1 %2B R_2}$$

Measuring the actual resistors used gave values of 98.3k ohms and 46k ohms which would result in an output of 3.12V at 10V input.

<DocsImageLayout :images="$frontmatter.images_divider" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 

## Wiring
<DocsImageLayout :images="$frontmatter.images_wiring" size="lg" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 

## Battery Interface
The voltage can be monitored conveniently using the balance pins.

<DocsImageLayout :images="$frontmatter.images_connector" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 