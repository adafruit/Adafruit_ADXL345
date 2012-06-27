#include <Wire.h>
#include <Adafruit_ADXL345.h>

Adafruit_ADXL345 adxl345;

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Hello!");
  
  Serial.println("Simple XYZ Accelerometer Test (ADXL345)");
  adxl345.begin();
  
  // Check connection
  uint8_t deviceid = adxl345.getDeviceID();
  if (deviceid != 0xE5)
  {
    Serial.print("Device ID: "); Serial.println(deviceid);
    Serial.println("Oops ... no ADXL345 detected.  Check your connections");
    // Wait around forever
    while(1);
  }
  else
  {
    Serial.println("Connected [device id = 0xE5]");
  }
}

void loop(void) 
{
  int16_t x, y, z;

  x = adxl345.getX();
  y = adxl345.getY();
  z = adxl345.getZ();
  Serial.print("X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.println(z);
  
  delay(100);
}