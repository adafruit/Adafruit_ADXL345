/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "Adafruit_ADXL345.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static uint8_t i2cread(void) {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static void i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
static void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value));
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
static uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 1);
  return (i2cread());  
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
static int16_t read16(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_ADDRESS, 2);
  return (uint16_t)(i2cread() | (i2cread() << 8));  
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
Adafruit_ADXL345::Adafruit_ADXL345() {
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void Adafruit_ADXL345::begin() {
  Wire.begin();
  
  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);  
}

/**************************************************************************/
/*! 
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t Adafruit_ADXL345::getDeviceID(void)
{
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345::getX(void) {
  return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345::getY(void) {
  return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345::getZ(void) {
  return read16(ADXL345_REG_DATAZ0);
}

