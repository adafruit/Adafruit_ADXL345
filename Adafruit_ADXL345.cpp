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
    @brief  Abstract away SPI receiver & transmitter
*/
/**************************************************************************/
static uint8_t spixfer(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t data) {
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(clock, LOW);
    digitalWrite(mosi, data & (1<<i));
    digitalWrite(clock, HIGH);
    if (digitalRead(miso)) 
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void  Adafruit_ADXL345::writeRegister(uint8_t reg, uint8_t value) {
  if (_i2c) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite((uint8_t)reg);
    i2cwrite((uint8_t)(value));
    Wire.endTransmission();
  } else {
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    spixfer(_clk, _di, _do, value);
    digitalWrite(_cs, HIGH);
  }
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t Adafruit_ADXL345::readRegister(uint8_t reg) {
  if (_i2c) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    return (i2cread());
  } else {
    reg |= 0x80; // read byte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint8_t reply = spixfer(_clk, _di, _do, 0xFF);
    digitalWrite(_cs, HIGH);
    return reply;
  }
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t  Adafruit_ADXL345::read16(uint8_t reg) {
  if (_i2c) {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 2);
    return (uint16_t)(i2cread() | (i2cread() << 8));  
  } else {
    reg |= 0x80 | 0x40; // read byte | multibyte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint16_t reply = spixfer(_clk, _di, _do, 0xFF)  | (spixfer(_clk, _di, _do, 0xFF) << 8);
    digitalWrite(_cs, HIGH);
    return reply;

  }
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
Adafruit_ADXL345::Adafruit_ADXL345() {
  _i2c = true;
}

Adafruit_ADXL345::Adafruit_ADXL345(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs) {
  _cs = cs;
  _clk = clock;
  _do = mosi;
  _di = miso;
  _i2c = false;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void Adafruit_ADXL345::begin() {
  if (_i2c)
    Wire.begin();
  else {
    pinMode(_cs, OUTPUT);
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, HIGH);
    pinMode(_do, OUTPUT);
    pinMode(_di, INPUT);
  }
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

