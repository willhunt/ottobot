---

---
# IMU
## Notes on usage of Adafruit BNO055
9DOF Internal Measurement Unit (IMU)

* [Datasheet](http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
* [Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)

## Specific use with Arduino M0 Pro
There are some specific notes for using the sensor with the Arduino M0 Pro
1. Use I2C address 0x29
    * e.g: `Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);`
2. Tie ADR pin to high