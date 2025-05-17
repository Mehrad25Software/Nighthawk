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
const int LINE_THRESHOLD = 1500;

// --- QTRSensors Object ---
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// --- Motor Control Configuration (using CytronMotorDriver Library) ---
#define MOTOR_LEFT_PWM_PIN   13  // Example ESP32 GPIO for Left Motor PWM (MDD10A PWM1)
#define MOTOR_LEFT_DIR_PIN   12  // Example ESP32 GPIO for Left Motor DIR (MDD10A DIR1)
#define MOTOR_RIGHT_PWM_PIN  14  // Example ESP32 GPIO for Right Motor PWM (MDD10A PWM2)
#define MOTOR_RIGHT_DIR_PIN  27  // Example ESP32 GPIO for Right Motor DIR (MDD10A DIR2)

CytronMD motorLeft(PWM_DIR, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN);
CytronMD motorRight(PWM_DIR, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN);

// Robot Speeds (0-255, the library handles negative for reverse)
#define ROBOT_SPEED_NORMAL          150 // Speed for normal forward movement
#define ROBOT_SPEED_TURN            130 // Speed for turning
#define ROBOT_SPEED_ESCAPE          180 // Speed for escaping when back sensor detects line
#define ROBOT_SPEED_REVERSE_SHORT   100 // Speed for short reverse maneuver

// Durations for maneuvers (in milliseconds)
#define REVERSE_DURATION_MS         200 // Duration for short reverse
#define TURN_DURATION_MS            250 // Duration for corrective turn

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
  Serial.print("Line Threshold (White): < ");
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
  digitalWrite(LED_BUILTIN, HIGH); 

  Serial.println("Setup complete. Monitoring sensors and controlling motors...");
  stop_motors(); 
}

// --- Motor Control Functions ---
void move_forward(int speed) {
  Serial.print("Motors: FORWARD, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(speed);
  motorRight.setSpeed(speed);
}

void move_backward(int speed) {
  Serial.print("Motors: BACKWARD, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(-speed);
  motorRight.setSpeed(-speed);
}

void turn_left(int speed) {
  Serial.print("Motors: TURN LEFT, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(-speed); 
  motorRight.setSpeed(speed);  
}

void turn_right(int speed) {
  Serial.print("Motors: TURN RIGHT, Speed: "); Serial.println(speed);
  motorLeft.setSpeed(speed);  
  motorRight.setSpeed(-speed); 
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
  if (leftFrontRawValue == 0 || rightFrontRawValue == 0 || backRawValue == 0) {
    Serial.println("! SENSOR(S) POSSIBLY DISCONNECTED - STOPPING !");
    if (leftFrontRawValue == 0) Serial.println("  -> Left Front sensor potentially disconnected.");
    if (rightFrontRawValue == 0) Serial.println("  -> Right Front sensor potentially disconnected.");
    if (backRawValue == 0) Serial.println("  -> Back sensor potentially disconnected.");
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
    move_backward(ROBOT_SPEED_TURN); 
    delay(200);                      
    turn_right(ROBOT_SPEED_TURN);    
    action_taken_this_cycle = true;
  }
  // Priority 4: Left front sensor detects white line (New Strategy)
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
  // Priority 5: Right front sensor detects white line (New Strategy)
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
    move_forward(ROBOT_SPEED_NORMAL);
    // action_taken_this_cycle remains false, or you can set it explicitly if needed for other logic
  }
  
  if (action_taken_this_cycle) {
    Serial.println();
  }

  delay(LOOP_DELAY); 
}
message.txt
8 KB
