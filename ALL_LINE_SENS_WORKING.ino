#include <QTRSensors.h> // Using the QTRSensors library

// --- Configuration ---
#define NUM_SENSORS        3  // We are using three QTR-1A sensors
#define EMITTER_PIN        255 // Use 255 for no explicit emitter control by ESP32
                               // (assuming QTR-1A LED is always on or controlled by a jumper)

// Define the pin for the built-in LED (usually GPIO 2 on many ESP32 boards)
// This is optional, can be removed if not needed for status.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// --- Sensor Pins ---
// IMPORTANT: Replace these with the actual ESP32 GPIO pins connected to your QTR-1A OUT pins.
// Ensure these are ADC1 pins (e.g., 32, 33, 34, 35, 36, 39).
const uint8_t QTR_PIN_LEFT_FRONT = 32;  // Example GPIO for Left Front Sensor
const uint8_t QTR_PIN_RIGHT_FRONT = 33; // Example GPIO for Right Front Sensor
const uint8_t QTR_PIN_BACK = 35;        // Example GPIO for Back Sensor

uint8_t sensorPins[NUM_SENSORS] = {QTR_PIN_LEFT_FRONT, QTR_PIN_RIGHT_FRONT, QTR_PIN_BACK};

// --- Line Detection Threshold ---
// Based on your observation:
// Raw value < 1500 is WHITE LINE (and not 0, which indicates disconnected)
// Raw value >= 1500 is BLACK BACKGROUND
const int LINE_THRESHOLD = 1500;

// --- QTRSensors Object ---
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS]; // To store raw sensor readings for all sensors

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { // Wait for serial, with a timeout
    delay(10);
  }

  Serial.println("\nESP32 Sumobot - QTR-1A Line Detection (3 Sensors)");
  Serial.println("-------------------------------------------------");
  Serial.print("Using QTRSensors Library. Number of sensors: ");
  Serial.println(NUM_SENSORS);
  Serial.print("Line Threshold (White): < ");
  Serial.print(LINE_THRESHOLD);
  Serial.println(" (and not 0 for disconnected)");
  Serial.println("Pins (Left_F, Right_F, Back): " + String(QTR_PIN_LEFT_FRONT) + ", " + String(QTR_PIN_RIGHT_FRONT) + ", " + String(QTR_PIN_BACK));
  Serial.println("-------------------------------------------------");
  Serial.println("Raw Value < " + String(LINE_THRESHOLD) + " AND Raw Value != 0 -> WHITE LINE");
  Serial.println("Raw Value == 0 -> Likely DISCONNECTED");
  Serial.println("Raw Value >= " + String(LINE_THRESHOLD) + " -> BLACK BACKGROUND");
  Serial.println("-------------------------------------------------");

  // Initialize QTRSensors
  qtr.setTypeAnalog(); // Set sensor type to analog
  qtr.setSamplesPerSensor(4); // Optional: average a few samples for stability.
                              // Remove or set to 1 for the absolute rawest single reading per sensor.
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN); // No explicit emitter control

  // No calibration sequence is run in this version, relying on the hardcoded threshold.

  // Optional: Use built-in LED to indicate the sketch is running
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
  Serial.println("Setup complete. Monitoring sensors...");
}

void loop() {
  // Read raw sensor values for all sensors
  // For ESP32 with QTRSensors in analog mode, this will typically populate
  // sensorValues with readings from 0 to 4095 (for 12-bit ADC).
  qtr.read(sensorValues); // Populates the sensorValues array

  // Access individual sensor values
  uint16_t leftFrontRawValue = sensorValues[0];  // Corresponds to QTR_PIN_LEFT_FRONT
  uint16_t rightFrontRawValue = sensorValues[1]; // Corresponds to QTR_PIN_RIGHT_FRONT
  uint16_t backRawValue = sensorValues[2];       // Corresponds to QTR_PIN_BACK

  // --- Optional: Print all raw sensor values for debugging ---
  /*
  Serial.print("Raw Values -> LeftF: ");
  Serial.print(leftFrontRawValue);
  Serial.print(" | RightF: ");
  Serial.print(rightFrontRawValue);
  Serial.print(" | Back: ");
  Serial.println(backRawValue);
  */

  // --- Line Detection Logic for each sensor ---

  // Check Left Front Sensor
  if (leftFrontRawValue == 0) {
    // Serial.print("LeftF Raw: 0");
    // Serial.println(" -> Left sensor possibly DISCONNECTED!");
  } else if (leftFrontRawValue < LINE_THRESHOLD) {
    Serial.print("LeftF Raw: "); Serial.print(leftFrontRawValue);
    Serial.println(" -> Left sensor detected: must turn right");
    // Add motor control logic here to turn right
  } else {
    // Sensor is on black background
    // Serial.print("LeftF Raw: "); Serial.print(leftFrontRawValue); Serial.println(" -> Black");
  }

  // Check Right Front Sensor
  if (rightFrontRawValue == 0) {
    // Serial.print("RightF Raw: 0");
    // Serial.println(" -> Right sensor possibly DISCONNECTED!");
  } else if (rightFrontRawValue < LINE_THRESHOLD) {
    Serial.print("RightF Raw: "); Serial.print(rightFrontRawValue);
    Serial.println(" -> Right sensor detected: must turn left");
    // Add motor control logic here to turn left
  } else {
    // Sensor is on black background
    // Serial.print("RightF Raw: "); Serial.print(rightFrontRawValue); Serial.println(" -> Black");
  }

  // Check Back Sensor
  if (backRawValue == 0) {
    // Serial.print("Back Raw: 0");
    // Serial.println(" -> Back sensor possibly DISCONNECTED!");
  } else if (backRawValue < LINE_THRESHOLD) {
    Serial.print("Back Raw: "); Serial.print(backRawValue);
    Serial.println(" -> Back sensor detected must move forward");
    // Add motor control logic here to move forward
  } else {
    // Sensor is on black background
    // Serial.print("Back Raw: "); Serial.print(backRawValue); Serial.println(" -> Black");
  }
  
  // Add a blank line in Serial Monitor for readability if any sensor triggered an action or was disconnected
  if ((leftFrontRawValue < LINE_THRESHOLD) || (rightFrontRawValue < LINE_THRESHOLD) || (backRawValue < LINE_THRESHOLD) ) {
      Serial.println(); 
  }


  delay(500); // Adjust delay as needed for your bot's responsiveness and serial output readability
}
