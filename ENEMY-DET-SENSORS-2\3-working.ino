#include <Wire.h>               // For I2C communication
#include <QTRSensors.h>         // For line sensors
#include <Adafruit_VL53L0X.h>   // For front IR ToF sensor (VL53L0X)
#include <Adafruit_VL6180X.h>   // For side IR ToF sensor (VL6180X)

// --- QTR Line Sensor Configuration ---
#define NUM_QTR_SENSORS    3
#define QTR_EMITTER_PIN    255 // No explicit emitter control

const uint8_t QTR_PIN_LEFT_FRONT = 32;
const uint8_t QTR_PIN_RIGHT_FRONT = 33;
const uint8_t QTR_PIN_BACK = 35;
uint8_t qtrSensorPins[NUM_QTR_SENSORS] = {QTR_PIN_LEFT_FRONT, QTR_PIN_RIGHT_FRONT, QTR_PIN_BACK};
const int LINE_THRESHOLD = 1500; // White line if raw value < 1500 (and not 0)
QTRSensors qtr;
uint16_t qtrSensorValues[NUM_QTR_SENSORS];

// --- IR ToF Enemy Sensor Configuration (Modified for 2 ToF Sensors) ---
// XSHUT (Shutdown) pins for ToF sensors
// IMPORTANT: Connect these to the specified GPIOs on your ESP32
const int XSHUT_PIN_SIDE_IR = 18;   // Example GPIO for the single Side VL6180X XSHUT
// const int XSHUT_PIN_RIGHT_IR = 19;  // Commented out - Right sensor not used for now
const int XSHUT_PIN_FRONT_IR = 23;  // Example GPIO for Front VL53L0X XSHUT

// New I2C addresses for VL6180X sensors (7-bit addresses)
// Default for VL6180X and VL53L0X is 0x29
#define VL6180X_SIDE_ADDR   0x30 // Address for the single side VL6180X
// #define VL6180X_RIGHT_ADDR  0x31 // Commented out
// VL53L0X will remain at its default address 0x29.

Adafruit_VL53L0X lox = Adafruit_VL53L0X();     // Front sensor object (VL53L0X)
Adafruit_VL6180X vl_side = Adafruit_VL6180X(); // Single Side sensor object (VL6180X)
// Adafruit_VL6180X vl_right = Adafruit_VL6180X(); // Commented out

// Enemy detection threshold (in millimeters)
const int ENEMY_DETECT_DISTANCE_FRONT_MM = 300;
const int ENEMY_DETECT_DISTANCE_SIDE_MM = 150; // VL6180X typically has a shorter max range (~100-200mm)


// Built-in LED (optional status indicator)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

void setup() {
  Serial.begin(115200);
  unsigned long setupStartTime = millis();
  while (!Serial && (millis() - setupStartTime < 3000)) { // Increased timeout for serial
    delay(10);
  }

  Serial.println("\nESP32 Sumobot - System Initialization (2 ToF Sensors)");
  Serial.println("-----------------------------------------------------");

  // --- Initialize Built-in LED ---
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Start with LED off, will turn on if all setup is OK

  // --- Initialize QTR Line Sensors ---
  Serial.println("Initializing QTR Line Sensors...");
  qtr.setTypeAnalog();
  qtr.setSamplesPerSensor(4);
  qtr.setSensorPins(qtrSensorPins, NUM_QTR_SENSORS);
  qtr.setEmitterPin(QTR_EMITTER_PIN);
  Serial.println("QTR Line Sensors Initialized.");
  Serial.print("Line Threshold (White): < ");
  Serial.print(LINE_THRESHOLD);
  Serial.println(" (and not 0 for disconnected)");
  Serial.println("-----------------------------------------------------");

  // --- Initialize I2C and ToF IR Sensors ---
  Serial.println("Initializing I2C and ToF IR Sensors (1x VL6180X Side, 1x VL53L0X Front)...");
  Wire.begin(); // Initialize I2C (SDA: GPIO21, SCL: GPIO22 on most ESP32s)

  // Setup XSHUT pins as OUTPUT
  pinMode(XSHUT_PIN_SIDE_IR, OUTPUT);
  // pinMode(XSHUT_PIN_RIGHT_IR, OUTPUT); // Commented out
  pinMode(XSHUT_PIN_FRONT_IR, OUTPUT);

  // Ensure all sensors are in shutdown initially
  digitalWrite(XSHUT_PIN_SIDE_IR, LOW);
  // digitalWrite(XSHUT_PIN_RIGHT_IR, LOW); // Commented out
  digitalWrite(XSHUT_PIN_FRONT_IR, LOW);
  Serial.println("All ToF sensors initially in shutdown mode.");
  delay(100); // Delay for shutdown to take effect

  bool ir_init_ok = true; // Flag to track overall IR sensor initialization

  // Initialize Side VL6180X Sensor (vl_side)
  Serial.println("--- Initializing Side VL6180X (ID: vl_side) ---");
  digitalWrite(XSHUT_PIN_SIDE_IR, HIGH); // Enable Side sensor
  delay(100); // Allow time for sensor to power up
  Serial.println("Attempting to begin Side VL6180X at default address 0x29...");
  if (!vl_side.begin()) {
    Serial.println("!!! ERROR: Failed to find Side VL6180X sensor at default address. Check wiring/XSHUT. !!!");
    ir_init_ok = false;
  } else {
    Serial.println("Side VL6180X found at default address. Setting new address...");
    vl_side.setAddress(VL6180X_SIDE_ADDR); // Change its address
    Serial.print("Side VL6180X address set to: 0x"); Serial.println(VL6180X_SIDE_ADDR, HEX);
  }


  // Initialize Front VL53L0X Sensor (lox)
  // The VL6180X should now be on its new address, and its XSHUT is HIGH.
  if (ir_init_ok) {
    Serial.println("--- Initializing Front VL53L0X (ID: lox) ---");
    digitalWrite(XSHUT_PIN_FRONT_IR, HIGH); // Enable Front sensor
    delay(100); // Allow time for sensor to power up
    Serial.println("Attempting to begin Front VL53L0X at default address 0x29...");
    if (!lox.begin()) { // Default address 0x29
      Serial.println("!!! ERROR: Failed to find Front VL53L0X sensor. Check wiring/XSHUT. !!!");
      ir_init_ok = false;
    } else {
      Serial.println("Front VL53L0X sensor found at default address 0x29.");
    }
  }

  Serial.println("-----------------------------------------------------");
  if (ir_init_ok) {
    Serial.println("ToF IR Sensor Initialization Process Completed Successfully.");
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED if all init steps were successful
  } else {
    Serial.println("!!! CRITICAL ERROR during ToF IR Sensor Initialization. Check Serial Log. !!!");
    digitalWrite(LED_BUILTIN, LOW); // Keep LED off
  }
  Serial.println("-----------------------------------------------------");
  Serial.println("Overall Setup Complete. Starting main loop if sensors are OK.");
}

void loop() {
  // --- 1. Line Detection (QTR Sensors) ---
  qtr.read(qtrSensorValues);
  uint16_t qtrLeftVal = qtrSensorValues[0];
  uint16_t qtrRightVal = qtrSensorValues[1];
  uint16_t qtrBackVal = qtrSensorValues[2];

  bool lineActionTaken = false; 

  if (qtrLeftVal == 0) {
    // Serial.println("QTR Left: DISCONNECTED!");
  } else if (qtrLeftVal < LINE_THRESHOLD) {
    Serial.print("QTR Left Raw: "); Serial.print(qtrLeftVal);
    Serial.println(" -> White Line (Left): Must turn right!");
    lineActionTaken = true;
    // Add motor_turn_right();
  }

  if (qtrRightVal == 0) {
    // Serial.println("QTR Right: DISCONNECTED!");
  } else if (qtrRightVal < LINE_THRESHOLD) {
    Serial.print("QTR Right Raw: "); Serial.print(qtrRightVal);
    Serial.println(" -> White Line (Right): Must turn left!");
    lineActionTaken = true;
    // Add motor_turn_left();
  }

  if (qtrBackVal == 0) {
    // Serial.println("QTR Back: DISCONNECTED!");
  } else if (qtrBackVal < LINE_THRESHOLD) {
    Serial.print("QTR Back Raw: "); Serial.print(qtrBackVal);
    Serial.println(" -> White Line (Back): Must move forward!");
    lineActionTaken = true;
    // Add motor_move_forward();
  }

  // --- 2. Enemy Detection (IR ToF Sensors) ---
  bool enemyActionTaken = false;

  // Read Front VL53L0X Sensor
  VL53L0X_RangingMeasurementData_t measure_front;
  lox.rangingTest(&measure_front, false); 

  if (measure_front.RangeStatus != 4 && measure_front.RangeStatus != 0 ) { 
    int dist_front_mm = measure_front.RangeMilliMeter;
    if (dist_front_mm > 0 && dist_front_mm < ENEMY_DETECT_DISTANCE_FRONT_MM) { 
      Serial.print("ENEMY FRONT! Dist: "); Serial.print(dist_front_mm); Serial.print(" mm (Status: "); Serial.print(measure_front.RangeStatus); Serial.println("). ATTACK!");
      enemyActionTaken = true;
      // Add motor_attack_forward();
    }
  }

  // Read Side VL6180X Sensor
  uint8_t dist_side_mm = vl_side.readRange();
  uint8_t status_side = vl_side.readRangeStatus();

  if (status_side == VL6180X_ERROR_NONE) {
    if (dist_side_mm > 0 && dist_side_mm < ENEMY_DETECT_DISTANCE_SIDE_MM) { 
        Serial.print("ENEMY SIDE! Dist: "); Serial.print(dist_side_mm); Serial.println(" mm. Turn towards enemy or evade!");
        enemyActionTaken = true;
        // Add motor_handle_side_enemy(); // You'll need to decide if this is left or right based on wiring
    }
  }

  // Logic for the second side sensor (vl_right) is commented out / removed
  /*
  uint8_t dist_right_mm = vl_right.readRange();
  uint8_t status_right = vl_right.readRangeStatus();

  if (status_right == VL6180X_ERROR_NONE) {
    if (dist_right_mm > 0 && dist_right_mm < ENEMY_DETECT_DISTANCE_SIDE_MM) {
        Serial.print("ENEMY RIGHT! Dist: "); Serial.print(dist_right_mm); Serial.println(" mm. Turn towards enemy or evade!");
        enemyActionTaken = true;
        // Add motor_handle_right_enemy();
    }
  }
  */

  if (lineActionTaken || enemyActionTaken) {
      Serial.println(); 
  }

  // --- Delay ---
  delay(100); 
}
