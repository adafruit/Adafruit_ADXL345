/**************************************************************************/
/*!
    @file     Adafruit_ADXL345_U.cpp
    @author   K.Townsend (Adafruit Industries)
*/
/**************************************************************************/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Adafruit_ADXL345_U.h"

/**************************************************************************/
/*!
    @brief  And abstract 'send' method for different versions of the Arduino
   Wire library
    @returns The byte read
*/
/**************************************************************************/
inline uint8_t Adafruit_ADXL345_Unified::i2cread(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

/**************************************************************************/
/*!
    @brief  And abstract 'send' method for different versions of the Arduino
   Wire library
    @param x The byte to send
*/
/**************************************************************************/
inline void Adafruit_ADXL345_Unified::i2cwrite(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

/**************************************************************************/

/**
 * @brief  Abstract away SPI receiver & transmitter

 * @param clock The pin number for SCK, the SPI ClocK line
 * @param miso The pin number for MISO, the SPI Master In Slave Out line
 * @param mosi The pin number for MOSI, the SPI Master Out Slave In line
 * @param data The byte to send
 *
 * @return uint8_t The single byte response
 */
static uint8_t spixfer(uint8_t clock, uint8_t miso, uint8_t mosi,
                       uint8_t data) {
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(clock, LOW);
    digitalWrite(mosi, data & (1 << i));
    digitalWrite(clock, HIGH);
    if (digitalRead(miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes one byte to the specified destination register
    @param reg The address of the register to write to
    @param value The value to set the register to
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::writeRegister(uint8_t reg, uint8_t value) {
  if (_i2c) {
    Wire.beginTransmission((uint8_t)_i2caddr);
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
    @brief Reads one byte from the specified register
    @param reg The address of the register to read from
    @returns The single byte value of the requested register
*/
/**************************************************************************/
uint8_t Adafruit_ADXL345_Unified::readRegister(uint8_t reg) {
  if (_i2c) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, 1);
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
    @brief Reads two bytes from the specified register
    @param reg The address of the register to read from
    @return The two bytes read from the sensor starting at the given address
*/
/**************************************************************************/
int16_t Adafruit_ADXL345_Unified::read16(uint8_t reg) {
  if (_i2c) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, 2);
    return (uint16_t)(i2cread() | (i2cread() << 8));
  } else {
    reg |= 0x80 | 0x40; // read byte | multibyte
    digitalWrite(_cs, LOW);
    spixfer(_clk, _di, _do, reg);
    uint16_t reply =
        spixfer(_clk, _di, _do, 0xFF) | (spixfer(_clk, _di, _do, 0xFF) << 8);
    digitalWrite(_cs, HIGH);
    return reply;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the device ID (can be used to check connection)
    @return The Device ID of the connected sensor
*/
/**************************************************************************/
uint8_t Adafruit_ADXL345_Unified::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent X axis value
    @return The raw `int16_t` unscaled x-axis acceleration value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345_Unified::getX(void) {
  return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Y axis value
    @return The raw `int16_t` unscaled y-axis acceleration value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345_Unified::getY(void) {
  return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent Z axis value
    @return The raw `int16_t` unscaled z-axis acceleration value
*/
/**************************************************************************/
int16_t Adafruit_ADXL345_Unified::getZ(void) {
  return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
    @param sensorID A unique ID to use to differentiate the sensor from others
*/
/**************************************************************************/
Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  _i2c = true;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class in SPI mode
    @param clock The pin number for SCK, the SPI ClocK line
    @param miso The pin number for MISO, the SPI Master In Slave Out line
    @param mosi The pin number for MOSI, the SPI Master Out Slave In line
    @param cs The pin number for CS, the SPI Chip Select line
    @param sensorID A unique ID to use to differentiate the sensor from others
*/
/**************************************************************************/
Adafruit_ADXL345_Unified::Adafruit_ADXL345_Unified(uint8_t clock, uint8_t miso,
                                                   uint8_t mosi, uint8_t cs,
                                                   int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
  _cs = cs;
  _clk = clock;
  _do = mosi;
  _di = miso;
  _i2c = false;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param i2caddr The I2C address to begin communication with
    @return true: success false: a sensor with the correct ID was not found
*/
/**************************************************************************/
bool Adafruit_ADXL345_Unified::begin(uint8_t i2caddr) {
  _i2caddr = i2caddr;

  if (_i2c)
    Wire.begin();
  else {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_clk, OUTPUT);
    digitalWrite(_clk, HIGH);
    pinMode(_do, OUTPUT);
    pinMode(_di, INPUT);
  }

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    return false;
  }

  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
    @param range The new `range_t` to set the accelerometer to
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setRange(range_t range) {
  /* Read the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
    @return The current `range_t` value
*/
/**************************************************************************/
range_t Adafruit_ADXL345_Unified::getRange(void) {
  /* Read the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
    @param dataRate The `dataRate_t` to set
*/
/**************************************************************************/
void Adafruit_ADXL345_Unified::setDataRate(dataRate_t dataRate) {
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
    @return The current data rate
*/
/**************************************************************************/
dataRate_t Adafruit_ADXL345_Unified::getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param event Pointer to the event object to fill
    @return true: success
*/
/**************************************************************************/
bool Adafruit_ADXL345_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;
  event->acceleration.x =
      getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y =
      getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z =
      getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
 */
/**************************************************************************/

/**
 * @brief Fill a `sensor_t` struct with information about the sensor
 *
 * @param sensor Pointer to a `sensor_t` struct to fill
 */
void Adafruit_ADXL345_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution = 0.03923F;  /*  4mg = 0.0392266 m/s^2 */
}
