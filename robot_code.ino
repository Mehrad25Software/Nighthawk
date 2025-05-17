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
// IMPORTANT: Replace with your actual ESP32 GPIO pins connected to MDD10A
// These pins will be used by the CytronMD objects.
#define MOTOR_LEFT_PWM_PIN   13  // Example ESP32 GPIO for Left Motor PWM (MDD10A PWM1)
#define MOTOR_LEFT_DIR_PIN   12  // Example ESP32 GPIO for Left Motor DIR (MDD10A DIR1)
#define MOTOR_RIGHT_PWM_PIN  14  // Example ESP32 GPIO for Right Motor PWM (MDD10A PWM2)
#define MOTOR_RIGHT_DIR_PIN  27  // Example ESP32 GPIO for Right Motor DIR (MDD10A DIR2)

// Create CytronMD objects for each motor
// The first argument PWM_DIR tells the library you're using PWM and Direction pins.
CytronMD motorLeft(PWM_DIR, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN);
CytronMD motorRight(PWM_DIR, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN);

// Robot Speeds (0-255, the library handles negative for reverse)
#define ROBOT_SPEED_NORMAL  150 // Speed for normal forward movement
#define ROBOT_SPEED_TURN    130 // Speed for turning
#define ROBOT_SPEED_ESCAPE  180 // Speed for escaping when back sensor detects line

// Loop delay
#define LOOP_DELAY          50 // Delay in milliseconds for the main loop

// --- Function Prototypes for Motor Control ---
void move_forward(int speed);
void move_backward(int speed);
void turn_left(int speed);
void turn_right(int speed);
void stop_motors();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    delay(10);
  }

  Serial.println("\nESP32 Sumobot - QTR-1A & CytronMotorDriver Library");
  Serial.println("----------------------------------------------------");
  Serial.print("Using QTRSensors Library. Number of sensors: ");
  Serial.println(NUM_SENSORS);
  Serial.print("Line Threshold (White): < ");
  Serial.println(LINE_THRESHOLD);
  Serial.println("Sensor Pins (Left_F, Right_F, Back): " + String(QTR_PIN_LEFT_FRONT) + ", " + String(QTR_PIN_RIGHT_FRONT) + ", " + String(QTR_PIN_BACK));
  Serial.println("Motor Pins (L_PWM, L_DIR, R_PWM, R_DIR): " + String(MOTOR_LEFT_PWM_PIN) + ", " + String(MOTOR_LEFT_DIR_PIN) + ", " + String(MOTOR_RIGHT_PWM_PIN) + ", " + String(MOTOR_RIGHT_DIR_PIN));
  Serial.println("----------------------------------------------------");

  // Initialize QTRSensors
  qtr.setTypeAnalog();
  qtr.setSamplesPerSensor(4);
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  // Motors are initialized when the CytronMD objects are created globally.
  // No explicit motor setup function is typically needed here with this library.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Setup complete. Monitoring sensors and controlling motors...");
  stop_motors(); // Ensure motors are stopped at startup
}

// --- Motor Control Functions using CytronMotorDriver library ---
// The library's setSpeed function takes an integer from -255 to 255.
// Positive for forward, negative for backward, 0 for stop.

void move_forward(int speed) {
  motorLeft.setSpeed(speed);
  motorRight.setSpeed(speed);
}

void move_backward(int speed) {
  motorLeft.setSpeed(-speed);
  motorRight.setSpeed(-speed);
}

void turn_left(int speed) {
  motorLeft.setSpeed(-speed); // Left motor backward
  motorRight.setSpeed(speed);  // Right motor forward
}

void turn_right(int speed) {
  motorLeft.setSpeed(speed);  // Left motor forward
  motorRight.setSpeed(-speed); // Right motor backward
}

void stop_motors() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

void loop() {
  qtr.read(sensorValues);
  uint16_t leftFrontRawValue = sensorValues[0];
  uint16_t rightFrontRawValue = sensorValues[1];
  uint16_t backRawValue = sensorValues[2];

  bool action_taken = true;

  if (leftFrontRawValue == 0 || rightFrontRawValue == 0 || backRawValue == 0) {
    Serial.println("! SENSOR(S) POSSIBLY DISCONNECTED - STOPPING !");
    if (leftFrontRawValue == 0) Serial.println("  -> Left Front sensor disconnected.");
    if (rightFrontRawValue == 0) Serial.println("  -> Right Front sensor disconnected.");
    if (backRawValue == 0) Serial.println("  -> Back sensor disconnected.");
    stop_motors();
  }
  else if (backRawValue < LINE_THRESHOLD) {
    Serial.print("Back Raw: "); Serial.print(backRawValue);
    Serial.println(" -> Back sensor detected WHITE: Moving FORWARD (Escape)");
    move_forward(ROBOT_SPEED_ESCAPE);
  }
  else if (leftFrontRawValue < LINE_THRESHOLD && rightFrontRawValue < LINE_THRESHOLD) {
    Serial.print("LF Raw: "); Serial.print(leftFrontRawValue);
    Serial.print(" | RF Raw: "); Serial.print(rightFrontRawValue);
    Serial.println(" -> Both Front sensors detected WHITE: Reversing and Turning Right");
    move_backward(ROBOT_SPEED_TURN);
    delay(200);
    turn_right(ROBOT_SPEED_TURN);
  }
  else if (leftFrontRawValue < LINE_THRESHOLD) {
    Serial.print("LeftF Raw: "); Serial.print(leftFrontRawValue);
    Serial.println(" -> Left sensor detected WHITE: Turning RIGHT");
    turn_right(ROBOT_SPEED_TURN);
  }
  else if (rightFrontRawValue < LINE_THRESHOLD) {
    Serial.print("RightF Raw: "); Serial.print(rightFrontRawValue);
    Serial.println(" -> Right sensor detected WHITE: Turning LEFT");
    turn_left(ROBOT_SPEED_TURN);
  }
  else {
    move_forward(ROBOT_SPEED_NORMAL);
    action_taken = false;
  }
  
  if (action_taken) {
    Serial.println();
  }

  delay(LOOP_DELAY);
}
