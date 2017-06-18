/*
  Using the AS726x Spectral Sensor
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 8th, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!

  Read the raw integer values from the sensor.
  Violet, Blue, Green, Yellow, Orange, and Red data from the AS7262 (Visible)
  R, S, T, U, V, W data from the AS7263 (NIR)

  The AS726x Qwiic board can be configured to communicate over I2C (default) or serial. This example
  assumes we are communicating over I2C. See schematic for jumpers to change to serial and datasheet
  for the AT command interface.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the AS726x onto the shield
  Serial.print it out at 115200 baud to serial monitor.

  Available:
  void takeMeasurements()
  void takeMeasurementsWithBulb()
  int getViolet() Blue() Green() Yellow() Orange() Red()
  float getCalibratedViolet(), Blue, Green, Yellow, Orange, Red
  void setMeasurementMode(byte mode)
  boolean dataAvailable()
  boolean as726xSetup()
  byte getTemperature()
  byte getTemperatureF()
  void setIndicatorCurrent(byte)
  void enableIndicator()
  void disableIndicator()
  void setBulbCurrent(byte)
  void enableBulb()
  void disableBulb()
  void softReset()
  setGain(byte gain)
  setIntegrationTime(byte integrationValue)
  enableInterrupt()
  disableInterrupt()
*/

#include <Wire.h>
#include <AS726xLib.h>

#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F

AS726x sensor = AS726x();

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("AS726x Read Example");

  Wire.begin();

  if (sensor.begin() == 0)
  {
    Serial.println("Sensor failed to respond. Check wiring.");
    while (1); //Freeze!
  }

  if(sensor.version() == SENSORTYPE_AS7262) Serial.println("AS7262 online!");
  if(sensor.version() == SENSORTYPE_AS7263) Serial.println("AS7263 online!");
}

void loop()
{
  sensor.takeMeasurements(); //No LED - easier on your eyes
  //sensor.takeMeasurementsWithBulb(); //Use LED - bright white LED

  float tempF = sensor.getTemperatureF();
  
  if(sensor.version() == SENSORTYPE_AS7262)
  {
    //Visible readings
    Serial.print(" Reading: V[");
    Serial.print(sensor.getViolet());
    Serial.print("] B[");
    Serial.print(sensor.getBlue());
    Serial.print("] G[");
    Serial.print(sensor.getGreen());
    Serial.print("] Y[");
    Serial.print(sensor.getYellow());
    Serial.print("] O[");
    Serial.print(sensor.getOrange());
    Serial.print("] R[");
    Serial.print(sensor.getRed());
  }
  else if(sensor.version() == SENSORTYPE_AS7263)
  {
    //Near IR readings
    Serial.print(" Reading: R[");
    Serial.print(sensor.getR());
    Serial.print("] S[");
    Serial.print(sensor.getS());
    Serial.print("] T[");
    Serial.print(sensor.getT());
    Serial.print("] U[");
    Serial.print(sensor.getU());
    Serial.print("] V[");
    Serial.print(sensor.getV());
    Serial.print("] W[");
    Serial.print(sensor.getW());
  }

  Serial.print("] tempF[");
  Serial.print(tempF, 1);
  Serial.print("]");

  Serial.println();

  //delay(500);
}
