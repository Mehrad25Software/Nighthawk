#include <Arduino.h>            // Explicitly include for Arduino core functions
#include <QTRSensors.h>         // Using the QTRSensors library
#include <CytronMotorDriver.h>  // Using Cytron's Motor Driver Library

// --- Configuration ---
#define NUM_SENSORS       3  // We are using three QTR-1A sensors
#define EMITTER_PIN       255 // Use 255 for no explicit emitter control by ESP32

// Define the pin for the built-in LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
                                                                                                      
// --- Sensor Pins ---
const uint8_t QTR_PIN_LEFT_FRONT = 32;
const uint8_t QTR_PIN_RIGHT_FRONT = 33;
const uint8_t QTR_PIN_BACK = 35;
uint8_t sensorPins[NUM_SENSORS] = {QTR_PIN_LEFT_FRONT, QTR_PIN_RIGHT_FRONT, QTR_PIN_BACK};

// --- Line Detection Threshold ---
const int LINE_THRESHOLD = 1500; // Assumes lower values mean white line

// --- QTRSensors Object ---
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// --- Motor Control Configuration (using CytronMotorDriver Library) ---
#define MOTOR_LEFT_PWM_PIN   13  // ESP32 GPIO for Left Motor PWM (MDD10A PWM1)
#define MOTOR_LEFT_DIR_PIN   12  // ESP32 GPIO for Left Motor DIR (MDD10A DIR1)
#define MOTOR_RIGHT_PWM_PIN  14  // ESP32 GPIO for Right Motor PWM (MDD10A PWM2)
#define MOTOR_RIGHT_DIR_PIN  27  // ESP32 GPIO for Right Motor DIR (MDD10A DIR2)

CytronMD motorLeft(PWM_DIR, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN);
CytronMD motorRight(PWM_DIR, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN);

// Robot Speeds (0-255, the library handles negative for reverse)
#define ROBOT_SPEED_NORMAL          150 // Speed for normal forward movement
#define ROBOT_SPEED_TURN            130 // Speed for turning
#define ROBOT_SPEED_ESCAPE          180 // Speed for escaping when back sensor detects line
#define ROBOT_SPEED_REVERSE_SHORT   100 // Speed for short reverse maneuver
#define MOTOR_TEST_SPEED             70 // Slow speed for testing motors in setup

// Durations for maneuvers (in milliseconds)
#define REVERSE_DURATION_MS         200 // Duration for short reverse
#define TURN_DURATION_MS            250 // Duration for corrective turn
#define MOTOR_TEST_DURATION_MS     2000 // Duration for each step in motor test sequence

// Loop delay
#define LOOP_DELAY                  50 // Delay in milliseconds for the main loop

// --- Function Prototypes for Motor Control ---
void move_forward(int speed);
void move_backward(int speed);
void turn_left(int speed);
void turn_right(int speed);
void stop_motors();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { // Wait for serial with a timeout
    delay(10);
  }

  Serial.println("\nESP32 Sumobot - QTR-1A & CytronMotorDriver Library (Updated Strategy)");
  Serial.println("-------------------------------------------------------------------");
  Serial.print("Using QTRSensors Library. Number of sensors: ");
  Serial.println(NUM_SENSORS);
  Serial.print("Line Threshold (White): < "); // Note: Verify this logic with your sensor readings
  Serial.println(LINE_THRESHOLD);
  Serial.println("Sensor Pins (Left_F, Right_F, Back): " + String(QTR_PIN_LEFT_FRONT) + ", " + String(QTR_PIN_RIGHT_FRONT) + ", " + String(QTR_PIN_BACK));
  Serial.println("Motor Pins (L_PWM, L_DIR, R_PWM, R_DIR): " + String(MOTOR_LEFT_PWM_PIN) + ", " + String(MOTOR_LEFT_DIR_PIN) + ", " + String(MOTOR_RIGHT_PWM_PIN) + ", " + String(MOTOR_RIGHT_DIR_PIN));
  Serial.println("-------------------------------------------------------------------");

  qtr.setTypeAnalog();
  qtr.setSamplesPerSensor(4);
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);
  Serial.println("QTR Sensors Initialized.");
  Serial.println("Cytron Motor Drivers Initialized (via global objects).");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate setup phase

  // +++ ADDED MOTOR TEST CODE START +++
  Serial.println("\n--- MOTOR TEST SEQUENCE ---");
  Serial.print("Testing motors: Both wheels FORWARD slowly at speed ");
  Serial.print(MOTOR_TEST_SPEED);
  Serial.print(" for ");
  Serial.print(MOTOR_TEST_DURATION_MS / 1000.0);
  Serial.println(" seconds...");
  move_forward(MOTOR_TEST_SPEED);
  delay(MOTOR_TEST_DURATION_MS);

  Serial.print("Testing motors: Both wheels BACKWARD slowly at speed ");
  Serial.print(MOTOR_TEST_SPEED);
  Serial.print(" for ");
  Serial.print(MOTOR_TEST_DURATION_MS / 1000.0);
  Serial.println(" seconds...");
  move_backward(MOTOR_TEST_SPEED);
  delay(MOTOR_TEST_DURATION_MS);

  Serial.print("Testing motors: Turning RIGHT slowly at speed ");
  Serial.print(MOTOR_TEST_SPEED);
  Serial.print(" for ");
  Serial.print( (MOTOR_TEST_DURATION_MS / 2) / 1000.0); // Shorter duration for turns
  Serial.println(" seconds...");
  turn_right(MOTOR_TEST_SPEED);
  delay(MOTOR_TEST_DURATION_MS / 2);

  Serial.print("Testing motors: Turning LEFT slowly at speed ");
  Serial.print(MOTOR_TEST_SPEED);
  Serial.print(" for ");
  Serial.print( (MOTOR_TEST_DURATION_MS / 2) / 1000.0); // Shorter duration for turns
  Serial.println(" seconds...");
  turn_left(MOTOR_TEST_SPEED);
  delay(MOTOR_TEST_DURATION_MS / 2);
  
  stop_motors(); // IMPORTANT: Stop motors after the test sequence
  Serial.println("--- MOTOR TEST SEQUENCE COMPLETE ---");
  Serial.println("If wheels did not turn as expected, please check:");
  Serial.println("  1. Motor power supply (battery, connections).");
  Serial.println("  2. Wiring between ESP32, motor driver (Cytron MDD10A), and motors.");
  Serial.println("  3. Motor driver health.");
  Serial.println("  4. Correct pin definitions for motors in the code.\n");
  // +++ ADDED MOTOR TEST CODE END +++

  Serial.println("Setup complete. Monitoring sensors and controlling motors...");
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED (or blink) to indicate operational mode
  stop_motors(); // Ensure motors are stopped before entering the main loop
}

// --- Motor Control Functions ---
void move_forward(int speed) {
  Serial.print("Motors: FORWARD, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(speed);
  motorRight.setSpeed(speed);
}

void move_backward(int speed) {
  Serial.print("Motors: BACKWARD, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(-speed); // Negative for reverse with Cytron library
  motorRight.setSpeed(-speed);// Negative for reverse with Cytron library
}

void turn_left(int speed) {
  Serial.print("Motors: TURN LEFT, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(-speed); // Left motor backward for pivot turn
  motorRight.setSpeed(speed);  // Right motor forward for pivot turn
}

void turn_right(int speed) {
  Serial.print("Motors: TURN RIGHT, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(speed);  // Left motor forward for pivot turn
  motorRight.setSpeed(-speed); // Right motor backward for pivot turn
}

void stop_motors() {
  Serial.println("Motors: STOP");
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void loop() {
  qtr.read(sensorValues); 
  uint16_t leftFrontRawValue = sensorValues[0];
  uint16_t rightFrontRawValue = sensorValues[1];
  uint16_t backRawValue = sensorValues[2];

  bool action_taken_this_cycle = false; 

  // Priority 1: Check for disconnected sensors
  // A raw value of 0 might indicate a disconnected analog sensor (floats low or problem with read)
  // This threshold (0) might need adjustment if 0 is a valid dark reading for your setup
  if (leftFrontRawValue == 0 || rightFrontRawValue == 0 || backRawValue == 0) {
    Serial.println("! SENSOR(S) POSSIBLY DISCONNECTED (value 0) - STOPPING !");
    if (leftFrontRawValue == 0) Serial.println("  -> Left Front sensor potentially problematic.");
    if (rightFrontRawValue == 0) Serial.println("  -> Right Front sensor potentially problematic.");
    if (backRawValue == 0) Serial.println("  -> Back sensor potentially problematic.");
    stop_motors();
    action_taken_this_cycle = true;
  }
  // Priority 2: Back sensor detects white line (escape maneuver)
  else if (backRawValue < LINE_THRESHOLD) {
    Serial.print("Back Raw: "); Serial.print(backRawValue);
    Serial.println(" -> Back sensor detected WHITE: Moving FORWARD (Escape)");
    move_forward(ROBOT_SPEED_ESCAPE);
    action_taken_this_cycle = true;
  }
  // Priority 3: Both front sensors detect white line (e.g., approaching edge head-on)
  else if (leftFrontRawValue < LINE_THRESHOLD && rightFrontRawValue < LINE_THRESHOLD) {
    Serial.print("LF Raw: "); Serial.print(leftFrontRawValue);
    Serial.print(" | RF Raw: "); Serial.print(rightFrontRawValue);
    Serial.println(" -> Both Front sensors detected WHITE: Reversing and Turning Right");
    move_backward(ROBOT_SPEED_REVERSE_SHORT); // Use ROBOT_SPEED_REVERSE_SHORT for consistency
    delay(REVERSE_DURATION_MS);               // Use defined REVERSE_DURATION_MS     
    turn_right(ROBOT_SPEED_TURN);    
    delay(TURN_DURATION_MS); // Added delay for turn completion
    action_taken_this_cycle = true;
  }
  // Priority 4: Left front sensor detects white line
  else if (leftFrontRawValue < LINE_THRESHOLD) {
    Serial.print("LeftF Raw: "); Serial.print(leftFrontRawValue);
    Serial.println(" -> Left sensor detected WHITE: Correcting Right");
    move_backward(ROBOT_SPEED_REVERSE_SHORT);
    delay(REVERSE_DURATION_MS);
    turn_right(ROBOT_SPEED_TURN);
    delay(TURN_DURATION_MS);
    move_forward(ROBOT_SPEED_NORMAL); // Resume forward after correction
    action_taken_this_cycle = true;
  }
  // Priority 5: Right front sensor detects white line
  else if (rightFrontRawValue < LINE_THRESHOLD) {
    Serial.print("RightF Raw: "); Serial.print(rightFrontRawValue);
    Serial.println(" -> Right sensor detected WHITE: Correcting Left");
    move_backward(ROBOT_SPEED_REVERSE_SHORT);
    delay(REVERSE_DURATION_MS);
    turn_left(ROBOT_SPEED_TURN);
    delay(TURN_DURATION_MS);
    move_forward(ROBOT_SPEED_NORMAL); // Resume forward after correction
    action_taken_this_cycle = true;
  }
  // Default: No line detected, move forward (search/normal operation)
  else {
    // Only print this if no specific line action was taken, to reduce serial spam
    if (!action_taken_this_cycle) {
       Serial.print("Default: No line. LF: "); Serial.print(leftFrontRawValue);
       Serial.print(", RF: "); Serial.print(rightFrontRawValue);
       Serial.print(", BK: "); Serial.println(backRawValue);
    }
    move_forward(ROBOT_SPEED_NORMAL);
    // action_taken_this_cycle remains false if this branch is hit directly
  }
  
  if (action_taken_this_cycle) {
    Serial.println(); // Adds a blank line after an action sequence for better log readability
  }

  delay(LOOP_DELAY); 
}
