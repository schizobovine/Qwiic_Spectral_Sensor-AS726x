/*
 * Using the AS726x Spectral Sensor and a Teensy
 *
 * Author: Sean Caulfield
 * License: GPLv2.0
 *
 */

#include <Arduino.h>
#include <Bounce2.h>
#include <AS726xTeensyLib.h>

// Teensy LC
//#define WIRE0_PINS     I2C_PINS_18_19
//#define WIRE1_PINS     I2C_PINS_22_23

// Teensy 3.2
#define WIRE0_PINS     I2C_PINS_18_19
#define WIRE1_PINS     I2C_PINS_29_30

#define I2C_SPEED      400000
#define BUTT_PIN       24
#define DEBOUNCE_MS    10
#define I2C_TIMEOUT_US 10000

AS726xTeensy sensor0 = AS726xTeensy();
AS726xTeensy sensor1 = AS726xTeensy();

//Bounce butt;

// Output visible readings
void output_vis(AS726xTeensy *sensor) {
  Serial.print(" Reading: V[");
  Serial.print(sensor->getViolet());
  Serial.print("] B[");
  Serial.print(sensor->getBlue());
  Serial.print("] G[");
  Serial.print(sensor->getGreen());
  Serial.print("] Y[");
  Serial.print(sensor->getYellow());
  Serial.print("] O[");
  Serial.print(sensor->getOrange());
  Serial.print("] R[");
  Serial.print(sensor->getRed());
  Serial.println("]");
}

// Output Near IR readings
void output_nir(AS726xTeensy *sensor) {
  Serial.print(" Reading: R[");
  Serial.print(sensor->getR());
  Serial.print("] S[");
  Serial.print(sensor->getS());
  Serial.print("] T[");
  Serial.print(sensor->getT());
  Serial.print("] U[");
  Serial.print(sensor->getU());
  Serial.print("] V[");
  Serial.print(sensor->getV());
  Serial.print("] W[");
  Serial.print(sensor->getW());
  Serial.println("]");
}

void init_sensor(AS726xTeensy *sensor, i2c_t3 *bus) {

  if (sensor->begin(bus) == 0)
  {
    Serial.println("sensor0 failed to respond. Check wiring.");
    while (1); //Freeze!
  }

  if (sensor->version() == SENSORTYPE_AS7262) {
    Serial.println("AS7262 online!");
  } else if (sensor->version() == SENSORTYPE_AS7263) {
    Serial.println("AS7263 online!");
  }

}

void setup()
{

  Serial.begin(115200);
  while (!Serial);
  Serial.println("AS726x Read Example");

  Serial.println("init Wire0");
  Wire.begin(I2C_MASTER, 0, WIRE0_PINS, I2C_PULLUP_EXT, I2C_SPEED, I2C_OP_MODE_IMM);
  Wire.setDefaultTimeout(I2C_TIMEOUT_US);
  Serial.println("init Wire1");
  Wire1.begin(I2C_MASTER, 0, WIRE1_PINS, I2C_PULLUP_EXT, I2C_SPEED, I2C_OP_MODE_IMM);
  Wire1.setDefaultTimeout(I2C_TIMEOUT_US);

  Serial.println("init sensor0");
  init_sensor(&sensor0, &Wire);
  Serial.println("init sensor1");
  init_sensor(&sensor1, &Wire1);

  pinMode(BUTT_PIN, INPUT_PULLUP);
  //butt.attach(BUTT_PIN, DEBOUNCE_MS);

}

void loop()
{

  //butt.update();

  if (digitalRead(BUTT_PIN) == LOW) {
    //Use LED - bright white LED
    sensor0.takeMeasurementsWithBulb();
    sensor1.takeMeasurementsWithBulb();
  } else {
    //No LED - easier on your eyes
    sensor0.takeMeasurements();
    sensor1.takeMeasurements();
  }


  // Output results from sensor0
  Serial.print("0: ");
  if(sensor0.version() == SENSORTYPE_AS7262) {
    output_vis(&sensor0);
  } else if(sensor0.version() == SENSORTYPE_AS7263) {
    output_nir(&sensor0);
  }
  
  // Output results from sensor1
  Serial.print("1: ");
  if(sensor1.version() == SENSORTYPE_AS7262) {
    output_vis(&sensor1);
  } else if(sensor1.version() == SENSORTYPE_AS7263) {
    output_nir(&sensor1);
  }

  //delay(500);
}
