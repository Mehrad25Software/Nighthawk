#include <Wire.h> // Included for completeness, though not directly used by QTR if I2C is not involved.
#include <QTRSensors.h>

// --- Configuration ---
#define NUM_SENSORS        1  // Number of sensors in your array (set to 1 for QTR-1A)
#define TIMEOUT            2500 // Waits for 2.5 ms for sensor outputs to go low (for RC type)
                               // For analog type, this is not directly used by read() but good to keep for library structure.

// Define the pin for the built-in LED (usually GPIO 2 on many ESP32 boards)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// EMITTER_PIN: ESP32 pin connected to the sensor array's LED control pin (CTRL, LEDON, EMITTER).
// Use 255 (which is what QTR_NO_EMITTER_PIN typically defines to) if your sensor
// doesn't have one, you're not controlling it, or it's a single module like QTR-1A
// where the LED is always on or controlled by a jumper.
#define EMITTER_PIN        255 // Using 255 directly for no emitter pin control.


// --- Sensor Pins ---
// Define the ESP32 GPIO pins connected to your QTR sensor outputs.
// GPIO 34 is an ADC1 pin on ESP32, suitable for an analog sensor like QTR-1A.
uint8_t sensorPins[NUM_SENSORS] = {
  34 // Single sensor connected to GPIO 34
};

// --- QTRSensors Object ---
// QTR-1A is an ANALOG sensor.
QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS]; // To store raw sensor readings
uint16_t calibratedSensorValues[NUM_SENSORS]; // To store calibrated sensor readings


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println("ESP32 QTR-1A Sensor Test - White Line on Black Background (Raw & Calibrated)");

  // --- Initialize QTRSensors ---
  qtr.setTypeAnalog(); // Specify sensor type as analog
  qtr.setSamplesPerSensor(4); // Number of analog readings to average per sensor (helps reduce noise)

  // Initialize with pins array, number of sensors, and emitter pin.
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN); // EMITTER_PIN is 255 for no explicit control.

  Serial.println("Calibrating sensor...");
  Serial.println("Please swipe the sensor over the WHITE LINE and BLACK BACKGROUND for 10-15 seconds.");

  // --- Calibration ---
  // During calibration, move the sensor over the lightest (white line) and darkest (black background) parts of the surface.
  pinMode(LED_BUILTIN, OUTPUT); // Use ESP32's built-in LED as a status indicator
  for (int i = 0; i < 250; i++) { // Calibrate for a certain duration (e.g., 250 * 20ms = 5 seconds)
    digitalWrite(LED_BUILTIN, HIGH);
    qtr.calibrate(); // Reads the sensor and updates calibration data
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Calibration complete.");
  Serial.print("Calibrated Min Value (should correspond to black background): ");
  if (qtr.calibrationOn.minimum) {
      Serial.println(qtr.calibrationOn.minimum[0]);
  } else {
      Serial.println("N/A (Calibration data not initialized)");
  }

  Serial.print("Calibrated Max Value (should correspond to white line): ");
  if (qtr.calibrationOn.maximum) {
      Serial.println(qtr.calibrationOn.maximum[0]);
  } else {
      Serial.println("N/A (Calibration data not initialized)");
  }
  Serial.println("--------------------------");
}

void loop() {
  // --- Read Raw Sensor Values ---
  // This reads the direct analog-to-digital converter (ADC) output.
  // For ESP32, this will typically be a value from 0 to 4095 (for 12-bit ADC).
  qtr.read(sensorValues); // Populates the sensorValues array with raw readings

  // --- Read Calibrated Sensor Value & Position ---
  // Use readLineWhite for a white line on a black background.
  uint16_t position = qtr.readLineWhite(calibratedSensorValues); // Populates calibratedSensorValues

  // --- Print Raw Sensor Value ---
  Serial.print("Raw Value: ");
  Serial.print(sensorValues[0]); // Print the raw ADC reading for the single sensor

  // --- Print Calibrated Sensor Value ---
  // calibratedSensorValues[0] will contain the reading.
  // For readLineWhite: ~0 for black background, ~1000 for white line.
  Serial.print(" | Calibrated Value: ");
  Serial.print(calibratedSensorValues[0]);

  // --- Print Line Position (less meaningful for a single sensor) ---
  Serial.print(" | Position: ");
  Serial.println(position);

  // Determine if the sensor is over the line based on a threshold
  // A common threshold is 500.
  // For a white line, a HIGHER calibrated value means it's on the line.
  if (calibratedSensorValues[0] > 500) { // Adjust threshold as needed
    Serial.println("Sensor is over the WHITE line.");
  } else {
    Serial.println("Sensor is over the BLACK background.");
  }
  Serial.println("--------------------------");

  delay(250); // Adjust delay as needed
}
