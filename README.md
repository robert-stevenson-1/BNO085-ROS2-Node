# BNO085-ROS2-Node
A ROS2 Compatible node that interfaces with the BNO085 IMU sensor via python

## Requirements
**TODO: Added Requirements.txt**

The node communicates with the BNO08x via i2c on the Raspberry Pi and Python.
The required software/libraries to run this code:
- [Adafruit_CircuitPython_BNO08x](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x)
- [Adafruit-extended-bus](https://github.com/adafruit/Adafruit_Python_Extended_Bus)
- [Adafruit_Blinka](https://github.com/adafruit/Adafruit_Blinka)
- [RPi GPIO](https://pypi.org/project/RPi.GPIO/)

## I2C Setup
The Raspberry Pi hardware I2C implementation doesn't handle clock stretching due to a [bug](http://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html).
This is a problem when using the BNO085 (and it's cousins it appears, the BNO55 and BNO080) because it causes the crashes when the sensor is communicating with the Pi's programs.

There are 2 recommended options across the internet:
- [Slow the I2C Clock](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching)
    - This didn't work well for me, as continues to crashed the script, or if too low wouldn't even detect the sensor.
    - Also follow the suggestion in this [hookup guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/python-circuitpython) and setting the Clock faster @400KHz made it worst and crashed more quickly
- Configure and Use [software I2C](https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md) instead which supports clock stretching at the cost of a slight increase in CPU usage.
    -  Using this [guide](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes/software-i2c) in addition to the previous link, I update the code the use the software I2C interface with `SDA` on `GPIO23`, and `SCL` on `GPIO24` and giving the bus id `3`

As an extra: `config.txt` on ubuntu on the Pi is located in `/boot/firmware/config.txt` instead of `/boot/config.txt`

## Test Setup
- Platform: Raspberry Pi 4
- OS: Ubuntu 22.04.2 LTS
- ROS: ROS2 Humble
- Python: 3.10.6
