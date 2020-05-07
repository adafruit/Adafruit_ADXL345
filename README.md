# Adafruit ADXL345 [![Build Status](https://github.com/adafruit/Adafruit_ADXL345/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_ADXL345/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_ADXL345/html/index.html)

This driver is for the Adafruit ADXL345 Breakout (http://www.adafruit.com/products/1231), and is based on Adafruit's Unified Sensor Library (Adafruit_Sensor).
Tested and works great with the Adafruit ADXL345 Breakout Board
[<img src="https://cdn-shop.adafruit.com/970x728/1231-00.jpg" width="500px">](https://www.adafruit.com/products/1231)
Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

## About the ADXL345 ##

The ADXL345 is a digital accelerometer that supports both SPI and I2C mode, with adjustable data rata and 'range' (+/-2/4/8/16g).  The Adafruit_ADXL345 driver takes advantage of I2C mode to reduce the total pin count required to use the sensor.

More information on the ADXL345 can be found in the datasheet: http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf

<!-- START COMPATIBILITY TABLE -->

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |      X       |             |            |
Atmega328 @ 12MHz  |      X       |             |            |
Atmega32u4 @ 16MHz |      X       |             |            |
Atmega32u4 @ 8MHz  |      X       |             |            |
ESP8266            |      X       |             |            |
Atmega2560 @ 16MHz |      X       |             |            |
ATSAM3X8E          |      X       |             |            |
ATSAM21D           |      X       |             |            |
ATtiny85 @ 16MHz   |             |      X       |            | sketch too big
ATtiny85 @ 8MHz    |             |      X       |            | sketch too big
Intel Curie @ 32MHz |      X       |             |            |
STM32F2            |             |             |     X       |

  * ATmega328 @ 16MHz : Arduino UNO, Adafruit Pro Trinket 5V, Adafruit Metro 328, Adafruit Metro Mini
  * ATmega328 @ 12MHz : Adafruit Pro Trinket 3V
  * ATmega32u4 @ 16MHz : Arduino Leonardo, Arduino Micro, Arduino Yun, Teensy 2.0
  * ATmega32u4 @ 8MHz : Adafruit Flora, Bluefruit Micro
  * ESP8266 : Adafruit Huzzah
  * ATmega2560 @ 16MHz : Arduino Mega
  * ATSAM3X8E : Arduino Due
  * ATSAM21D : Arduino Zero, M0 Pro
  * ATtiny85 @ 16MHz : Adafruit Trinket 5V
  * ATtiny85 @ 8MHz : Adafruit Gemma, Arduino Gemma, Adafruit Trinket 3V

<!-- END COMPATIBILITY TABLE -->
## What is the Adafruit Unified Sensor Library? ##

The Adafruit Unified Sensor Library (https://github.com/adafruit/Adafruit_Sensor) provides a common interface and data type for any supported sensor.  It defines some basic information about the sensor (sensor limits, etc.), and returns standard SI units of a specific type and scale for each supported sensor type.

It provides a simple abstraction layer between your application and the actual sensor HW, allowing you to drop in any comparable sensor with only one or two lines of code to change in your project (essentially the constructor since the functions to read sensor data and get information about the sensor are defined in the base Adafruit_Sensor class).

This is imporant useful for two reasons:

1.) You can use the data right away because it's already converted to SI units that you understand and can compare, rather than meaningless values like 0..1023.

2.) Because SI units are standardised in the sensor library, you can also do quick sanity checks working with new sensors, or drop in any comparable sensor if you need better sensitivity or if a lower cost unit becomes available, etc.

Light sensors will always report units in lux, gyroscopes will always report units in rad/s, etc. ... freeing you up to focus on the data, rather than digging through the datasheet to understand what the sensor's raw numbers really mean.

## About this Driver ##

Adafruit invests time and resources providing this open source code.  Please support Adafruit and open-source hardware by purchasing products from Adafruit!
# Dependencies
 * [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor)

# Contributing
Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_ADXL345/blob/master/CODE_OF_CONDUCT.md>)
before contributing to help this project stay welcoming.

## Documentation and doxygen
Documentation is produced by doxygen. Contributions should include documentation for any new code added.

Some examples of how to use doxygen can be found in these guide pages:

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips

Written by Kevin (KTOWN) Townsend for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit ADXL345" and install the library.
