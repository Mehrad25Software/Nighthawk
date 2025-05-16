#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// VL53L0X TOF Sensor setup

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  Serial.println("Starting VL53L0X and Line Sensor...");

  // I2C for TOF
  Wire.begin();

  // TOF sensor init
  if (!lox.begin()) {
    Serial.println(F("Failed to start VL53L0X. Check wiring."));
    while (1);
  }
  Serial.println(F("VL53L0X ready"));

  pinMode(LINE_SENSOR_PIN, INPUT);  // Line sensor as input
}

void loop() {
  // --- TOF Distance Measurement ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    Serial.print("Distance (mm): ");
    Serial.print(measure.RangeMilliMeter);
    Serial.print(" | Signal Rate (Mcps): ");
    Serial.print(measure.SignalRateRtnMegaCps, 4);
    Serial.print(" | Status: ");
    Serial.println(measure.RangeStatus);
  } else {
    Serial.println("Out of range or no target detected");
  }
