/*
 * AS726xLib.cpp - Library code for AS726xTeensy
 *
 * Author: Nathan Seidle (SparkFun), modifications to library form by Sean Caulfield
 * License: Public domain (but you should buy Nate a beer or just something from SparkFun)
 *
 */

#include<AS726xTeensyLib.h>

//Sets up the sensor for constant read
//Returns the sensor version (AS7262 or AS7263)
uint8_t AS726xTeensy::begin(void)
{
  uint8_t _sensorVersion = virtualReadRegister(AS726X_HW_VERSION);
  if (_sensorVersion != SENSORTYPE_AS7262 && _sensorVersion != SENSORTYPE_AS7263)
  {
    Serial.print("ID (should be 0x3E or 0x3F): 0x");
    Serial.println(_sensorVersion, HEX);
    return (false); //Device ID should be 0x3E
  }

  setBulbCurrent(0b00); //Set to 12.5mA (minimum)
  disableBulb(); //Turn off to avoid heating the sensor

  setIndicatorCurrent(0b11); //Set to 8mA (maximum)
  disableIndicator(); //Turn off lights to save power
  
  setIntegrationTime(50); //50 * 2.8ms = 140ms. 0 to 255 is valid.
  //If you use Mode 2 or 3 (all the colors) then integration time is double. 140*2 = 280ms between readings.

  setGain(3); //Set gain to 64x

  setMeasurementMode(3); //One-shot reading of VBGYOR

  this->sensorVersion = _sensorVersion;
  return (_sensorVersion);
}

// Advanced setup: use alternate i2c busen and setup for constant read
// Returns the sensor version (AS7262 or AS7263)
uint8_t AS726xTeensy::begin(i2c_t3 *_bus)
{
  this->bus = _bus;
  return AS726xTeensy::begin();
}

// Return which type of hardware we are
uint8_t AS726xTeensy::version(void)
{
  return this->sensorVersion;
}

//Sets the measurement mode
//Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
//Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
//Mode 2: Continuous reading of all channels (power-on default)
//Mode 3: One-shot reading of all channels
void AS726xTeensy::setMeasurementMode(uint8_t mode)
{
  if(mode > AS726X_MODE_ONESHOT) mode = AS726X_MODE_ONESHOT;

  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP); //Read
  value &= 0b11110011; //Clear BANK bits
  value |= (mode << 2); //Set BANK bits with user's choice
  virtualWriteRegister(AS726X_CONTROL_SETUP, value); //Write
}

//Sets the gain value
//Gain 0: 1x (power-on default)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
void AS726xTeensy::setGain(uint8_t gain)
{
  if(gain > AS726X_GAIN_64X) gain = AS726X_GAIN_64X;

  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP); //Read
  value &= 0b11001111; //Clear GAIN bits
  value |= (gain << 4); //Set GAIN bits with user's choice
  virtualWriteRegister(AS726X_CONTROL_SETUP, value); //Write
}

//Sets the integration value
//Give this function a byte from 0 to 255.
//Time will be 2.8ms * [integration value]
void AS726xTeensy::setIntegrationTime(uint8_t integrationValue)
{
  virtualWriteRegister(AS726X_INT_T, integrationValue); //Write
}

//Enables the interrupt pin
void AS726xTeensy::enableInterrupt()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP); //Read
  value |= 0b01000000; //Set INT bit
  virtualWriteRegister(AS726X_CONTROL_SETUP, value); //Write
}

//Disables the interrupt pin
void AS726xTeensy::disableInterrupt()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP); //Read
  value &= 0b10111111; //Clear INT bit
  virtualWriteRegister(AS726X_CONTROL_SETUP, value); //Write
}

//Tells IC to take measurements and polls for data ready flag
void AS726xTeensy::takeMeasurements()
{
  clearDataAvailable(); //Clear DATA_RDY flag when using Mode 3

  //Goto mode 3 for one shot measurement of all channels
  setMeasurementMode(3);

  //Wait for data to be ready
  while(dataAvailable() == false) delay(AS726X_POLLING_DELAY);

  //Readings can now be accessed via getViolet(), getBlue(), etc
}

//Turns on bulb, takes measurements, turns off bulb
void AS726xTeensy::takeMeasurementsWithBulb()
{
  //enableIndicator(); //Tell the world we are taking a reading. 
  //The indicator LED is red and may corrupt the readings

  enableBulb(); //Turn on bulb to take measurement

  takeMeasurements();
  
  disableBulb(); //Turn off bulb to avoid heating sensor
  //disableIndicator();
}

//A the 16-bit value stored in a given channel registerReturns 
int AS726xTeensy::getChannel(uint8_t channelRegister)
{
  int colorData = virtualReadRegister(channelRegister) << 8; //High byte
  colorData |= virtualReadRegister(channelRegister + 1); //Low byte
  return(colorData);
}

//Given an address, read four bytes and return the floating point calibrated value
float AS726xTeensy::getCalibratedValue(uint8_t calAddress)
{
  uint8_t b0, b1, b2, b3;
  b0 = virtualReadRegister(calAddress + 0);
  b1 = virtualReadRegister(calAddress + 1);
  b2 = virtualReadRegister(calAddress + 2);
  b3 = virtualReadRegister(calAddress + 3);

  //Channel calibrated values are stored big-endian
  uint32_t calBytes = 0;
  calBytes |= ((uint32_t)b0 << (8 * 3));
  calBytes |= ((uint32_t)b1 << (8 * 2));
  calBytes |= ((uint32_t)b2 << (8 * 1));
  calBytes |= ((uint32_t)b3 << (8 * 0));

  return (convertBytesToFloat(calBytes));
}

//Given 4 bytes returns the floating point value
float AS726xTeensy::convertBytesToFloat(uint32_t myLong)
{
  float myFloat;
  memcpy(&myFloat, &myLong, 4); //Copy bytes into a float
  return (myFloat);
}

//Checks to see if DRDY flag is set in the control setup register
boolean AS726xTeensy::dataAvailable()
{
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP);
  return (value & (1 << 1)); //Bit 1 is DATA_RDY
}

//Clears the DRDY flag
//Normally this should clear when data registers are read
boolean AS726xTeensy::clearDataAvailable()
{
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP);
  value &= ~(1 << 1); //Set the DATA_RDY bit
  virtualWriteRegister(AS726X_CONTROL_SETUP, value);
}

//Enable the onboard indicator LED
void AS726xTeensy::enableIndicator()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL);
  value |= (1 << 0); //Set the bit
  virtualWriteRegister(AS726X_LED_CONTROL, value);
}

//Disable the onboard indicator LED
void AS726xTeensy::disableIndicator()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL);
  value &= ~(1 << 0); //Clear the bit
  virtualWriteRegister(AS726X_LED_CONTROL, value);
}

//Set the current limit of onboard LED. Default is max 8mA = 0b11.
void AS726xTeensy::setIndicatorCurrent(uint8_t current)
{
  if (current > 0b11) current = 0b11;
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL); //Read
  value &= 0b11111001; //Clear ICL_IND bits
  value |= (current << 1); //Set ICL_IND bits with user's choice
  virtualWriteRegister(AS726X_LED_CONTROL, value); //Write
}

//Enable the onboard 5700k or external incandescent bulb
void AS726xTeensy::enableBulb()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL);
  value |= (1 << 3); //Set the bit
  virtualWriteRegister(AS726X_LED_CONTROL, value);
}

//Disable the onboard 5700k or external incandescent bulb
void AS726xTeensy::disableBulb()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL);
  value &= ~(1 << 3); //Clear the bit
  virtualWriteRegister(AS726X_LED_CONTROL, value);
}

//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
void AS726xTeensy::setBulbCurrent(uint8_t current)
{
  if (current > 0b11) current = 0b11; //Limit to two bits

  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_LED_CONTROL); //Read
  value &= 0b11001111; //Clear ICL_DRV bits
  value |= (current << 4); //Set ICL_DRV bits with user's choice
  virtualWriteRegister(AS726X_LED_CONTROL, value); //Write
}

//Returns the temperature in C
//Pretty inaccurate: +/-8.5C
uint8_t AS726xTeensy::getTemperature()
{
  return (virtualReadRegister(AS726X_DEVICE_TEMP));
}
//Convert to F if needed
float AS726xTeensy::getTemperatureF()
{
  float temperatureF = getTemperature();
  temperatureF = temperatureF * 1.8 + 32.0;
  return (temperatureF);
}

//Does a soft reset
//Give sensor at least 1000ms to reset
void AS726xTeensy::softReset()
{
  //Read, mask/set, write
  uint8_t value = virtualReadRegister(AS726X_CONTROL_SETUP); //Read
  value |= (1<<7); //Set RST bit
  virtualWriteRegister(AS726X_CONTROL_SETUP, value); //Write
}

//Read a virtual register from the AS726xTeensy
uint8_t AS726xTeensy::virtualReadRegister(uint8_t virtualAddr)
{
  uint8_t status;

  //Do a prelim check of the read register
  status = readRegister(AS72XX_SLAVE_STATUS_REG);
  if ((status & AS72XX_SLAVE_RX_VALID) != 0) //There is data to be read
  {
    //Serial.println("Premptive read");
    uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG); //Read the byte but do nothing with it
  }

  //Wait for WRITE flag to clear
  while (1)
  {
    status = readRegister(AS72XX_SLAVE_STATUS_REG);
    if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
    delay(AS726X_POLLING_DELAY);
  }

  // Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
  writeRegister(AS72XX_SLAVE_WRITE_REG, virtualAddr);

  //Wait for READ flag to be set
  while (1)
  {
    status = readRegister(AS72XX_SLAVE_STATUS_REG);
    if ((status & AS72XX_SLAVE_RX_VALID) != 0) break; // Read data is ready.
    delay(AS726X_POLLING_DELAY);
  }

  uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG);
  return (incoming);
}

//Write to a virtual register in the AS726xTeensy
void AS726xTeensy::virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite)
{
  uint8_t status;

  //Wait for WRITE register to be empty
  while (1)
  {
    status = readRegister(AS72XX_SLAVE_STATUS_REG);
    if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
    delay(AS726X_POLLING_DELAY);
  }

  // Send the virtual register address (setting bit 7 to indicate we are writing to a register).
  writeRegister(AS72XX_SLAVE_WRITE_REG, (virtualAddr | 0x80));

  //Wait for WRITE register to be empty
  while (1)
  {
    status = readRegister(AS72XX_SLAVE_STATUS_REG);
    if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
    delay(AS726X_POLLING_DELAY);
  }

  // Send the data to complete the operation.
  writeRegister(AS72XX_SLAVE_WRITE_REG, dataToWrite);
}

//Get the various color readings (AS7262)
int AS726xTeensy::getViolet()             { return(getChannel(AS7262_V)); }
int AS726xTeensy::getBlue()               { return(getChannel(AS7262_B)); }
int AS726xTeensy::getGreen()              { return(getChannel(AS7262_G)); }
int AS726xTeensy::getYellow()             { return(getChannel(AS7262_Y)); }
int AS726xTeensy::getOrange()             { return(getChannel(AS7262_O)); }
int AS726xTeensy::getRed()                { return(getChannel(AS7262_R)); }

//Get the various NIR readings (AS7263)
int AS726xTeensy::getR()                  { return(getChannel(AS7263_R)); }
int AS726xTeensy::getS()                  { return(getChannel(AS7263_S)); }
int AS726xTeensy::getT()                  { return(getChannel(AS7263_T)); }
int AS726xTeensy::getU()                  { return(getChannel(AS7263_U)); }
int AS726xTeensy::getV()                  { return(getChannel(AS7263_V)); }
int AS726xTeensy::getW()                  { return(getChannel(AS7263_W)); }

//Returns the various calibration data for AS7262 visible light
float AS726xTeensy::getCalibratedViolet() { return(getCalibratedValue(AS7262_V_CAL)); }
float AS726xTeensy::getCalibratedBlue()   { return(getCalibratedValue(AS7262_B_CAL)); }
float AS726xTeensy::getCalibratedGreen()  { return(getCalibratedValue(AS7262_G_CAL)); }
float AS726xTeensy::getCalibratedYellow() { return(getCalibratedValue(AS7262_Y_CAL)); }
float AS726xTeensy::getCalibratedOrange() { return(getCalibratedValue(AS7262_O_CAL)); }
float AS726xTeensy::getCalibratedRed()    { return(getCalibratedValue(AS7262_R_CAL)); }

//Returns the various calibration data for AS7263 NIR
float AS726xTeensy::getCalibratedR()      { return(getCalibratedValue(AS7263_R_CAL)); }
float AS726xTeensy::getCalibratedS()      { return(getCalibratedValue(AS7263_S_CAL)); }
float AS726xTeensy::getCalibratedT()      { return(getCalibratedValue(AS7263_T_CAL)); }
float AS726xTeensy::getCalibratedU()      { return(getCalibratedValue(AS7263_U_CAL)); }
float AS726xTeensy::getCalibratedV()      { return(getCalibratedValue(AS7263_V_CAL)); }
float AS726xTeensy::getCalibratedW()      { return(getCalibratedValue(AS7263_W_CAL)); }

// Return all raw data in a single structure

as726x_raw_vis_t *AS726xTeensy::getAll(as726x_raw_vis_t *dest) {
  if (!dest) return NULL;
  dest->vis_v = this->getViolet();
  dest->vis_b = this->getBlue();
  dest->vis_g = this->getGreen();
  dest->vis_y = this->getYellow();
  dest->vis_o = this->getOrange();
  dest->vis_r = this->getRed();
  return dest;
}

as726x_raw_nir_t *AS726xTeensy::getAll(as726x_raw_nir_t *dest) {
  if (!dest) return NULL;
  dest->nir_r = this->getR();
  dest->nir_s = this->getS();
  dest->nir_t = this->getT();
  dest->nir_u = this->getU();
  dest->nir_v = this->getV();
  dest->nir_w = this->getW();
  return dest;
}

as726x_cal_vis_t *AS726xTeensy::getCalibratedAll(as726x_cal_vis_t *dest) {
  if (!dest) return NULL;
  dest->vis_v = this->getCalibratedViolet();
  dest->vis_b = this->getCalibratedBlue();
  dest->vis_g = this->getCalibratedGreen();
  dest->vis_y = this->getCalibratedYellow();
  dest->vis_o = this->getCalibratedOrange();
  dest->vis_r = this->getCalibratedRed();
  return dest;
}

as726x_cal_nir_t *AS726xTeensy::getCalibratedAll(as726x_cal_nir_t *dest) {
  if (!dest) return NULL;
  dest->nir_r = this->getCalibratedR();
  dest->nir_s = this->getCalibratedS();
  dest->nir_t = this->getCalibratedT();
  dest->nir_u = this->getCalibratedU();
  dest->nir_v = this->getCalibratedV();
  dest->nir_w = this->getCalibratedW();
  return dest;
}

//Reads from a give location from the AS726xTeensy
uint8_t AS726xTeensy::readRegister(uint8_t addr)
{
  this->bus->beginTransmission(AS726X_ADDR);
  this->bus->write(addr);
  this->bus->endTransmission();

  this->bus->requestFrom(AS726X_ADDR, 1);
  if (this->bus->available()) return (this->bus->read());

  Serial.println("I2C Error");
  return (0xFF); //Error
}

//Write a value to a spot in the AS726xTeensy
void AS726xTeensy::writeRegister(uint8_t addr, uint8_t val)
{
  this->bus->beginTransmission(AS726X_ADDR);
  this->bus->write(addr);
  this->bus->write(val);
  this->bus->endTransmission();
}
