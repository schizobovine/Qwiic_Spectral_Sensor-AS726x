/*
 * Using the AS726x Spectral Sensor and a Teensy and a SSD1331 color OLED
 *
 * Author: Sean Caulfield
 * License: GPLv2.0
 *
 */

#include <Arduino.h>
#include <AS726xTeensyLib.h>
#include <SSD_13XX.h>

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

#define OLED_CS  10
#define OLED_DC  9

// 
#define COUNTS_FLUX_VIS (0.45)
#define COUNTS_FLUX_NIR (0.35)

AS726xTeensy vis = AS726xTeensy();
AS726xTeensy nir = AS726xTeensy();

SSD_13XX oled = SSD_13XX(OLED_CS, OLED_DC);

as726x_raw_vis_t raw_vis;
as726x_raw_nir_t raw_nir;
as726x_cal_vis_t cal_vis;
as726x_cal_nir_t cal_nir;



// Output visible readings
void output_vis(as726x_raw_vis_t *raw_vis, as726x_cal_vis_t *cal_vis) {

  Serial.print("VIS (raw): V[");
  Serial.print(raw_vis->vis_v);
  Serial.print("] B[");
  Serial.print(raw_vis->vis_b);
  Serial.print("] G[");
  Serial.print(raw_vis->vis_g);
  Serial.print("] Y[");
  Serial.print(raw_vis->vis_y);
  Serial.print("] O[");
  Serial.print(raw_vis->vis_o);
  Serial.print("] R[");
  Serial.print(raw_vis->vis_r);
  Serial.println("]");

  Serial.print("VIS (cal): V[");
  Serial.print(cal_vis->vis_v);
  Serial.print("] B[");
  Serial.print(cal_vis->vis_b);
  Serial.print("] G[");
  Serial.print(cal_vis->vis_g);
  Serial.print("] Y[");
  Serial.print(cal_vis->vis_y);
  Serial.print("] O[");
  Serial.print(cal_vis->vis_o);
  Serial.print("] R[");
  Serial.print(cal_vis->vis_r);
  Serial.println("]");

}

// Output Near IR readings
void output_nir(as726x_raw_nir_t *raw_nir, as726x_cal_nir_t *cal_nir) {

  Serial.print("NIR (raw): R[");
  Serial.print(raw_nir->nir_r);
  Serial.print("] S[");
  Serial.print(raw_nir->nir_s);
  Serial.print("] T[");
  Serial.print(raw_nir->nir_t);
  Serial.print("] U[");
  Serial.print(raw_nir->nir_u);
  Serial.print("] V[");
  Serial.print(raw_nir->nir_v);
  Serial.print("] W[");
  Serial.print(raw_nir->nir_w);
  Serial.println("]");

  Serial.print("NIR (cal): R[");
  Serial.print(cal_nir->nir_r);
  Serial.print("] S[");
  Serial.print(cal_nir->nir_s);
  Serial.print("] T[");
  Serial.print(cal_nir->nir_t);
  Serial.print("] U[");
  Serial.print(cal_nir->nir_u);
  Serial.print("] V[");
  Serial.print(cal_nir->nir_v);
  Serial.print("] W[");
  Serial.print(cal_nir->nir_w);
  Serial.println("]");

}

void display_vis(AS726xTeensy *sensor) {
}

void display_nir(AS726xTeensy *sensor) {
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

  sensor->setGain(AS726X_GAIN_16X);

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

  Serial.println("init AS7262 (VIS)");
  init_sensor(&vis, &Wire);

  Serial.println("init AS7262 (NIR)");
  init_sensor(&nir, &Wire1);

  pinMode(BUTT_PIN, INPUT_PULLUP);
  
  Serial.println("init oled");
  oled.begin();
  uint8_t errorCode = oled.getErrorCode();
  if (errorCode != 0) {
    if ((errorCode & 1) == 0) {
      Serial.println("MOSI/SCLK pin mismatch");
    } else if ((errorCode & 1) == 0) {
      Serial.println("CS/DC pin mismatch");
    }
    while (1);
  }

  oled.changeMode(DISP_ON);
  oled.clearScreen();
  /*
  oled.fillRect(0, 0, 96, 64, BLACK);
  delay(200);
  oled.fillRect(0, 0, 96, 64, RED);
  delay(200);
  oled.fillRect(0, 0, 96, 64, ORANGE);
  delay(200);
  oled.fillRect(0, 0, 96, 64, YELLOW);
  delay(200);
  oled.fillRect(0, 0, 96, 64, GREEN);
  delay(200);
  oled.fillRect(0, 0, 96, 64, BLUE);
  delay(200);
  oled.fillRect(0, 0, 96, 64, MAGENTA);
  delay(200);
  oled.fillRect(0, 0, 96, 64, WHITE);
  */

}

void loop()
{

  if (digitalRead(BUTT_PIN) == LOW) {
    //Use LED - bright white LED
    vis.takeMeasurementsWithBulb();
    nir.takeMeasurementsWithBulb();
  } else {
    //No LED - easier on your eyes
    vis.takeMeasurements();
    nir.takeMeasurements();
  }

  // Store results
  vis.getAll(&raw_vis);
  nir.getAll(&raw_nir);
  vis.getCalibratedAll(&cal_vis);
  nir.getCalibratedAll(&cal_nir);

  // Output results from visible sensor
  output_vis(&raw_vis, &cal_vis);
  
  // Output results from NIR sensor
  output_nir(&raw_nir, &cal_nir);

}
