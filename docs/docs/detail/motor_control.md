---
images_encoder: [
    {src: "ottobot_encoder_test2.png", caption: "Logic analyser response over 10 wheel rotations"},
]
images_encoderleft: [
    {src: "detail_encoder_left_forward.png", caption: "Left wheel forwards"},
    {src: "detail_encoder_left_backward.png", caption: "Left wheel backwards"},
]
images_encoderright: [
    {src: "detail_encoder_right_forward.png", caption: "Right wheel forwards"},
    {src: "detail_encoder_right_backward.png", caption: "Right wheel backwards"},
]
images_wiring: [
    {src: "detail_motor_wiring.png", caption: "Motor control wiring"},
]
---
# Motor Control

## M0 Pro Considerations
[M0 Pro specification](https://store.arduino.cc/m0-pro)
### PWM
Pins 2 - 13 can be used for PWM with the analogWrite() function with the exceptions that pairs 4 & 10 and 5 & 12 cannot be used together.

### Interrupts
Interrupts are available on all pins except pin 4.

## Motor Encoders
From the [product page](http://www.dagurobot.com/DAGU%20_Wild_Thumper_%20encoders_RS003_Enco_with_motor_34_1?search=encoder&category_id=0):

> Encoders use custom made 8-pole magnets and 2 hall-effect sensors to provide 2 square waves, 90 degree out of phase with a total of 16 state changes per motor revolution.

There is then a 75:1 gear ratio meaning that **one revolution results in 1200 state changes**.

This was checked using a logic analyser and [PulseView](https://sigrok.org/wiki/PulseView). At the time a 34:1 gearbox was on the motor which should result in 544 edges per rotation. The wheel was rotated 10 times and the rising/falling edges counted. The count total was 5466 which is 546.6 per revolution, very close to the calculated amount (wheel cannot accurately be stopped at start position).

<DocsImageLayout :images="$frontmatter.images_encoder" size="lg" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 

### Direction
To determine both the speed and direction, the change in just one encoder phase can be used to increment a counter. On each count the corresponding level of the other phase can be used to determine direction. Using the logic analyzer we can inspect this.

<DocsImageLayout :images="$frontmatter.images_encoderleft" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 
Here we are looking at the left side encoder. When driving the robot forwards and a rising edge is detected from the yellow (coloured wire) phase, the red phase reading in LOW. When running backwards a rising edge on the yellow phase corresponds to a reading of HIGH on the red phase. 

<DocsImageLayout :images="$frontmatter.images_encoderright" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 
Here we are looking at the right side encoder. When driving the robot forwards and a rising edge is detected from the yellow (coloured wire) phase, the red phase reading in HIGH. When running backwards a rising edge on the yellow phase corresponds to a reading of LOW on the red phase. As expected the left and right sides are opposite as the motors are rotated 180 degrees.

On the Arduino this can then be used to increment or decrement a counter to determine fractional rotation forwards or backwards. For the left side in the `setup()` section:
```cpp
attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_A), update_wheel_tick_left, RISING);
```
Then in the interrupt service routine `update_wheel_tick_left`, called on the rising edge of `PIN_ENCODER_LEFT_A`:
```cpp
void update_wheel_tick_left() {
    // Look at other phase to determine direction
    if (digitalRead(PIN_ENCODER_LEFT_B) == LOW) {
        ticks_left ++;  // Forwards
    } else {
        ticks_left --;  // Backwards
    }
}
```

## Wiring
<DocsImageLayout :images="$frontmatter.images_wiring" size="lg" srcBase="/ottobot/assets/detail/"></DocsImageLayout> 