#include <QTRSensors.h> // We can still use the library for its read() function and setup.

// --- Configuration ---
#define NUM_SENSORS        1  // Single QTR-1A sensor
#define EMITTER_PIN        255 // Use 255 for no explicit emitter control by ESP32
                               // (assuming QTR-1A LED is always on or controlled by a jumper)

// Define the pin for the built-in LED (usually GPIO 2 on many ESP32 boards)
// This is optional, can be removed if not needed for status.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// --- Sensor Pin ---
// GPIO 34 is an ADC1 pin on ESP32, suitable for the QTR-1A analog sensor.
uint8_t sensorPins[NUM_SENSORS] = {34};

// --- Hardcoded Threshold ---
// Based on your observation:
// Raw value < 1500 is WHITE LINE
// Raw value >= 1500 is BLACK BACKGROUND
const int LINE_THRESHOLD = 1500;

// --- QTRSensors Object ---
// We still use the QTRSensors object to easily read the analog value,
// especially if we want to use its averaging feature.
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS]; // To store raw sensor readings

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println("QTR-1A Line Detection - Hardcoded Threshold");
  Serial.print("Using threshold: ");
  Serial.println(LINE_THRESHOLD);
  Serial.println("-------------------------------------------");
  Serial.println("Raw Value < 1500 -> WHITE LINE");
  Serial.println("Raw Value >= 1500 -> BLACK BACKGROUND");
  Serial.println("-------------------------------------------");


  // Initialize QTRSensors (even for raw reading, this sets up pins and type)
  qtr.setTypeAnalog();
  qtr.setSamplesPerSensor(4); // Optional: average a few samples for stability.
                              // Remove or set to 1 if you prefer the absolute rawest single reading.
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  // No calibration needed in this version.

  // Optional: Use built-in LED to indicate the sketch is running
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
}

void loop() {
  // Read raw sensor values
  // For ESP32, this will typically be a value from 0 to 4095 (for 12-bit ADC).
  qtr.read(sensorValues); // Populates the sensorValues array with raw readings
  uint16_t currentRawValue = sensorValues[0];

  Serial.print("Raw Value: ");
  Serial.print(currentRawValue);

  // Determine if the sensor is over the white line or black background
  if (currentRawValue < LINE_THRESHOLD) {
    Serial.println(" -> WHITE LINE detected");
    // Add your bot's logic here for when the white line is detected
    // e.g., turn away, stop, adjust motors, etc.
  } else {
    Serial.println(" -> BLACK BACKGROUND detected");
    // Add your bot's logic here for when the black background is detected
    // e.g., continue forward, etc.
  }

  delay(100); // Adjust delay as needed for your bot's responsiveness
}
