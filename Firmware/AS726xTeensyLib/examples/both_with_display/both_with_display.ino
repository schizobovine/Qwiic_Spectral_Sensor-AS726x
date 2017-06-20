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
#include "_fonts/Terminal_9.c"

#ifdef DEBUG
#define DPRINT(...) DPRINT(__VA_ARGS__)
#define DPRINTLN(...) DPRINTLN(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

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

// Counts per uW/cm^2 at 16X gain
// 1 uW/cm^2 = 0.01 W/m^2
#define COUNTS_FLUX_VIS (45.0)
#define COUNTS_FLUX_NIR (35.0)

AS726xTeensy vis = AS726xTeensy();
AS726xTeensy nir = AS726xTeensy();

SSD_13XX oled = SSD_13XX(OLED_CS, OLED_DC);

as726x_raw_vis_t raw_vis;
as726x_raw_nir_t raw_nir;
as726x_cal_vis_t cal_vis;
as726x_cal_nir_t cal_nir;

// Output visible readings
void output_vis(as726x_raw_vis_t *raw_vis, as726x_cal_vis_t *cal_vis) {

  DPRINT("VIS (raw): V[");
  DPRINT(raw_vis->vis_v);
  DPRINT("] B[");
  DPRINT(raw_vis->vis_b);
  DPRINT("] G[");
  DPRINT(raw_vis->vis_g);
  DPRINT("] Y[");
  DPRINT(raw_vis->vis_y);
  DPRINT("] O[");
  DPRINT(raw_vis->vis_o);
  DPRINT("] R[");
  DPRINT(raw_vis->vis_r);
  DPRINTLN("]");

  DPRINT("VIS (cal): V[");
  DPRINT(cal_vis->vis_v);
  DPRINT("] B[");
  DPRINT(cal_vis->vis_b);
  DPRINT("] G[");
  DPRINT(cal_vis->vis_g);
  DPRINT("] Y[");
  DPRINT(cal_vis->vis_y);
  DPRINT("] O[");
  DPRINT(cal_vis->vis_o);
  DPRINT("] R[");
  DPRINT(cal_vis->vis_r);
  DPRINTLN("]");

}

// Output Near IR readings
void output_nir(as726x_raw_nir_t *raw_nir, as726x_cal_nir_t *cal_nir) {

  DPRINT("NIR (raw): R[");
  DPRINT(raw_nir->nir_r);
  DPRINT("] S[");
  DPRINT(raw_nir->nir_s);
  DPRINT("] T[");
  DPRINT(raw_nir->nir_t);
  DPRINT("] U[");
  DPRINT(raw_nir->nir_u);
  DPRINT("] V[");
  DPRINT(raw_nir->nir_v);
  DPRINT("] W[");
  DPRINT(raw_nir->nir_w);
  DPRINTLN("]");

  DPRINT("NIR (cal): R[");
  DPRINT(cal_nir->nir_r);
  DPRINT("] S[");
  DPRINT(cal_nir->nir_s);
  DPRINT("] T[");
  DPRINT(cal_nir->nir_t);
  DPRINT("] U[");
  DPRINT(cal_nir->nir_u);
  DPRINT("] V[");
  DPRINT(cal_nir->nir_v);
  DPRINT("] W[");
  DPRINT(cal_nir->nir_w);
  DPRINTLN("]");

}

// Display legend, gain, etc
void display_other(void) {

  oled.setTextColor(WHITE, BLACK);
  oled.setCursor(16, 0);
  oled.print("VIS");
  oled.setCursor(60, 0);
  oled.print("NIR");

  oled.setTextColor(BLACK, PURPLE);
  oled.setCursor(0, 10);
  oled.print("V");
  oled.setCursor(88, 10);
  oled.print("R");

  oled.setTextColor(BLACK, BLUE);
  oled.setCursor(0, 18);
  oled.print("B");
  oled.setCursor(88, 18);
  oled.print("S");

  oled.setTextColor(BLACK, GREEN);
  oled.setCursor(0, 26);
  oled.print("G");
  oled.setCursor(88, 26);
  oled.print("T");

  oled.setTextColor(BLACK, YELLOW);
  oled.setCursor(0, 34);
  oled.print("Y");
  oled.setCursor(88, 34);
  oled.print("U");

  oled.setTextColor(BLACK, ORANGE);
  oled.setCursor(0, 42);
  oled.print("O");
  oled.setCursor(88, 42);
  oled.print("V");

  oled.setTextColor(BLACK, RED);
  oled.setCursor(0, 50);
  oled.print("R");
  oled.setCursor(88, 50);
  oled.print("W");

  //oled.setTextColor(WHITE, BLACK);
  //oled.setCursor(26, 56);
  //oled.print("W/m^2");

  oled.drawFastHLine(0, 8, 96, WHITE);
  oled.drawFastVLine(48, 0, 56, WHITE);

}

void display_vis(as726x_raw_vis_t *raw_vis, as726x_cal_vis_t *cal_vis) {

  oled.setTextColor(WHITE, WHITE);
  oled.fillRect(10, 10, 38, 56, BLACK);

  oled.setCursor(10, 10);
  oled.print(cal_vis->vis_v, 0);
  oled.setCursor(10, 18);
  oled.print(cal_vis->vis_b, 0);
  oled.setCursor(10, 26);
  oled.print(cal_vis->vis_g, 0);
  oled.setCursor(10, 34);
  oled.print(cal_vis->vis_y, 0);
  oled.setCursor(10, 42);
  oled.print(cal_vis->vis_o, 0);
  oled.setCursor(10, 50);
  oled.print(cal_vis->vis_r, 0);

}

void display_nir(as726x_raw_nir_t *raw_nir, as726x_cal_nir_t *cal_nir) {

  oled.setTextColor(WHITE, WHITE);
  oled.fillRect(49, 10, 38, 56, BLACK);

  oled.setCursor(49, 10);
  oled.print(cal_nir->nir_r, 0);
  oled.setCursor(49, 18);
  oled.print(cal_nir->nir_s, 0);
  oled.setCursor(49, 26);
  oled.print(cal_nir->nir_t, 0);
  oled.setCursor(49, 34);
  oled.print(cal_nir->nir_u, 0);
  oled.setCursor(49, 42);
  oled.print(cal_nir->nir_v, 0);
  oled.setCursor(49, 49);
  oled.print(cal_nir->nir_w, 0);

}

void init_sensor(AS726xTeensy *sensor, i2c_t3 *bus) {

  if (sensor->begin(bus) == 0)
  {
    DPRINTLN("sensor0 failed to respond. Check wiring.");
    while (1); //Freeze!
  }

  if (sensor->version() == SENSORTYPE_AS7262) {
    DPRINTLN("AS7262 online!");
  } else if (sensor->version() == SENSORTYPE_AS7263) {
    DPRINTLN("AS7263 online!");
  }

  sensor->setGain(AS726X_GAIN_16X);

}

void setup()
{

  Serial.begin(115200);
  while (!Serial);
  DPRINTLN("AS726x Read Example");

  DPRINTLN("init Wire0");
  Wire.begin(I2C_MASTER, 0, WIRE0_PINS, I2C_PULLUP_EXT, I2C_SPEED, I2C_OP_MODE_IMM);
  Wire.setDefaultTimeout(I2C_TIMEOUT_US);

  DPRINTLN("init Wire1");
  Wire1.begin(I2C_MASTER, 0, WIRE1_PINS, I2C_PULLUP_EXT, I2C_SPEED, I2C_OP_MODE_IMM);
  Wire1.setDefaultTimeout(I2C_TIMEOUT_US);

  DPRINTLN("init AS7262 (VIS)");
  init_sensor(&vis, &Wire);

  DPRINTLN("init AS7262 (NIR)");
  init_sensor(&nir, &Wire1);

  pinMode(BUTT_PIN, INPUT_PULLUP);
  
  DPRINTLN("init oled");
  oled.begin();
  uint8_t errorCode = oled.getErrorCode();
  if (errorCode != 0) {
    if ((errorCode & 1) == 0) {
      DPRINTLN("MOSI/SCLK pin mismatch");
    } else if ((errorCode & 1) == 0) {
      DPRINTLN("CS/DC pin mismatch");
    }
    while (1);
  }

  oled.changeMode(DISP_ON);
  oled.clearScreen();
  oled.setFont(&Terminal_9);
  oled.setTextWrap(false);
  oled.setCharSpacing(1);

  display_other();

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
  //output_vis(&raw_vis, &cal_vis);
  display_vis(&raw_vis, &cal_vis);
  
  // Output results from NIR sensor
  //output_nir(&raw_nir, &cal_nir);
  display_nir(&raw_nir, &cal_nir);

}
