/*
 * AS726xLib.h - C Header file for AS726xLib library code.
 *
 * Author: Nathan Seidle (SparkFun), modifications to library form by Sean Caulfield
 * License: Public domain (but you should buy Nate a beer or just something from SparkFun)
 *
 */

#ifndef __AS726X_LIB_H__
#define __AS726X_LIB_H__

//7-bit unshifted default I2C Address
#define AS726X_ADDR 0x49

//Register addresses
#define AS726X_DEVICE_TYPE      0x00
#define AS726X_HW_VERSION       0x01
#define AS726X_CONTROL_SETUP    0x04
#define AS726X_INT_T            0x05
#define AS726X_DEVICE_TEMP      0x06
#define AS726X_LED_CONTROL      0x07

#define AS72XX_SLAVE_STATUS_REG 0x00
#define AS72XX_SLAVE_WRITE_REG  0x01
#define AS72XX_SLAVE_READ_REG   0x02

//The same register locations are shared between the AS7262 and AS7263, they're
//just called something different

//AS7262 Registers
#define AS7262_V     0x08
#define AS7262_B     0x0A
#define AS7262_G     0x0C
#define AS7262_Y     0x0E
#define AS7262_O     0x10
#define AS7262_R     0x12
#define AS7262_V_CAL 0x14
#define AS7262_B_CAL 0x18
#define AS7262_G_CAL 0x1C
#define AS7262_Y_CAL 0x20
#define AS7262_O_CAL 0x24
#define AS7262_R_CAL 0x28

//AS7263 Registers
#define AS7263_R     0x08
#define AS7263_S     0x0A
#define AS7263_T     0x0C
#define AS7263_U     0x0E
#define AS7263_V     0x10
#define AS7263_W     0x12
#define AS7263_R_CAL 0x14
#define AS7263_S_CAL 0x18
#define AS7263_T_CAL 0x1C
#define AS7263_U_CAL 0x20
#define AS7263_V_CAL 0x24
#define AS7263_W_CAL 0x28

#define AS72XX_SLAVE_TX_VALID 0x02
#define AS72XX_SLAVE_RX_VALID 0x01

#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F

// Mode control
#define AS726X_MODE_CONT_BANK1 0x00 // Continuous reading of VBGY (7262) / STUV (7263)
#define AS726X_MODE_CONT_BANK2 0x01 // Continuous reading of VBGY (7262) / STUV (7263)
#define AS726X_MODE_CONT_ALL   0x02 // Continuous reading of all channels (power-on default)
#define AS726X_MODE_ONESHOT    0x03 // One-shot reading of all channels

// Gain control
#define AS726X_GAIN_1X  0x00 //Gain 0: 1x (power-on default)
#define AS726X_GAIN_4X  0x00 //Gain 1: 3.7x
#define AS726X_GAIN_16X 0x00 //Gain 2: 16x
#define AS726X_GAIN_64X 0x00 //Gain 3: 64x

//Amount of ms to wait between checking for virtual register changes
#define AS726X_POLLING_DELAY 5

class AS726x
{

  public:

    //Sets up the sensor for constant read
    //Returns the sensor version (AS7262 or AS7263)
    uint8_t begin(void);

    // Returns sensor version
    uint8_t version();

    //Sets the measurement mode
    //Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
    //Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
    //Mode 2: Continuous reading of all channels (power-on default)
    //Mode 3: One-shot reading of all channels
    void setMeasurementMode(uint8_t mode);

    //Sets the gain value
    //Gain 0: 1x (power-on default)
    //Gain 1: 3.7x
    //Gain 2: 16x
    //Gain 3: 64x
    void setGain(uint8_t gain);

    //Sets the integration value
    //Give this function a byte from 0 to 255.
    //Time will be 2.8ms * [integration value]
    void setIntegrationTime(uint8_t integrationValue);

    //Enables the interrupt pin
    void enableInterrupt();

    //Disables the interrupt pin
    void disableInterrupt();

    //Tells IC to take measurements and polls for data ready flag
    void takeMeasurements();

    //Turns on bulb, takes measurements, turns off bulb
    void takeMeasurementsWithBulb();

    //A the 16-bit value stored in a given channel registerReturns 
    int getChannel(uint8_t channelRegister);

    //Get the various color readings
    int getViolet();
    int getBlue();
    int getGreen();
    int getYellow();
    int getOrange();
    int getRed();

    //Get the various NIR readings
    int getR();
    int getS();
    int getT();
    int getU();
    int getV();
    int getW();

    //Given an address, read four bytes and return the floating point calibrated value
    float getCalibratedValue(uint8_t calAddress);

    //Returns the various calibration data for visible light
    float getCalibratedViolet();
    float getCalibratedBlue();
    float getCalibratedGreen();
    float getCalibratedYellow();
    float getCalibratedOrange();
    float getCalibratedRed();

    //Returns the various calibration data for NIR
    float getCalibratedR();
    float getCalibratedS();
    float getCalibratedT();
    float getCalibratedU();
    float getCalibratedV();
    float getCalibratedW();

    //Given 4 bytes returns the floating point value
    float convertBytesToFloat(uint32_t myLong);

    //Checks to see if DRDY flag is set in the control setup register
    boolean dataAvailable();

    //Clears the DRDY flag
    //Normally this should clear when data registers are read
    boolean clearDataAvailable();

    //Enable the onboard indicator LED
    void enableIndicator();

    //Disable the onboard indicator LED
    void disableIndicator();

    //Set the current limit of onboard LED. Default is max 8mA = 0b11.
    void setIndicatorCurrent(uint8_t current);

    //Enable the onboard 5700k or external incandescent bulb
    void enableBulb();

    //Disable the onboard 5700k or external incandescent bulb
    void disableBulb();

    //Set the current limit of bulb/LED.
    //Current 0: 12.5mA
    //Current 1: 25mA
    //Current 2: 50mA
    //Current 3: 100mA
    void setBulbCurrent(uint8_t current);

    //Returns the temperature in C
    //Pretty inaccurate: +/-8.5C
    uint8_t getTemperature();

    //Convert to F if needed
    float getTemperatureF();

    //Does a soft reset
    //Give sensor at least 1000ms to reset
    void softReset();

  protected:

    //Read a virtual register from the AS726x
    uint8_t virtualReadRegister(uint8_t virtualAddr);

    //Write to a virtual register in the AS726x
    void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite);

    //Reads from a give location from the AS726x
    uint8_t readRegister(uint8_t addr);

    //Write a value to a spot in the AS726x
    void writeRegister(uint8_t addr, uint8_t val);

  private:

    uint8_t sensorVersion = 0;

};

#endif
