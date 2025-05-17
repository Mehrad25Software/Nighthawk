#include <MyMotorControl.h>
#include <QTRSensors.h> // Using the QTRSensors library
// --- Configuration ---

#define NUM_SENSORS 3 // We are using three QTR-1A sensors
//#define EMITTER_PIN 255
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const uint8_t QTR_PIN_LEFT_FRONT = 32; // Example GPIO for Left Front Sensor
const uint8_t QTR_PIN_RIGHT_FRONT = 33; // Example GPIO for Right Front Sensor
const uint8_t QTR_PIN_BACK = 35; // Example GPIO for Back Sensor

#define MOTOR_LEFT_PWM_PIN   13
#define MOTOR_LEFT_DIR_PIN   12
#define MOTOR_RIGHT_PWM_PIN  14
#define MOTOR_RIGHT_DIR_PIN  27
#define IR_PIN               23
// Create an instance of your motor control library
MyMotorControl motors(MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN);

uint8_t sensorPins[NUM_SENSORS] = {QTR_PIN_LEFT_FRONT, QTR_PIN_RIGHT_FRONT, QTR_PIN_BACK};

const int LINE_THRESHOLD = 1500;
bool isRunning = false;
bool currentIRState;
bool previousIRState;
QTRSensors qtr;

void handleStartStopSensor() {
  bool currentIRState = digitalRead(IR_PIN);
  // detect rising edge (LOW -> HIGH)
  if (currentIRState == HIGH && previousIRState == LOW) {
    isRunning = true;
  }
  // detect falling edge (HIGH -> LOW)
  if (currentIRState == LOW && previousIRState == HIGH) {
    isRunning = false;
  }
  // update previous state for next loop
  previousIRState = currentIRState;
}

uint16_t sensorValues[NUM_SENSORS]; // To store raw sensor readings for all sensors

void setup() {
  qtr.setTypeAnalog(); // Set sensor type to analog
  qtr.setSamplesPerSensor(3); // Optional: average a few samples for stability.
  // Remove or set to 1 for the absolute rawest single reading per sensor.
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
//  qtr.setEmitterPin(EMITTER_PIN); // Specify that ESP32 isn't controlling an emitter pin.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
  Serial.println("Setup complete. Monitoring sensors...");
  while(!isRunning){
    handleStartStopSensor();
    delay(10);
  }
}

int main(){
  handleStartStopSensor();
  if (!isRunning) return 1;
  motors.moveForward(160);
  qtr.read(sensorValues); // Populates the sensorValues array
  // Access individual sensor values
  uint16_t leftFrontRawValue = sensorValues[0]; // Corresponds to QTR_PIN_LEFT_FRONT
  uint16_t rightFrontRawValue = sensorValues[1]; // Corresponds to QTR_PIN_RIGHT_FRONT
  uint16_t backRawValue = sensorValues[2]; // Corresponds to QTR_PIN_BACK
  // Check Left Front Sensor
  if (leftFrontRawValue < LINE_THRESHOLD) {
  // Add motor control logic here to turn right
    motors.moveBackward(120);
    delay(1500);
    motors.setLeftSpeed(180);
    motors.setRightSpeed(100);
    delay(1000);
    motors.moveForward(160);
  }
  // If leftFrontRawValue == 0, it's ignored by the logic above.
  // Check Right Front Sensor
  if (rightFrontRawValue < LINE_THRESHOLD) {
    motors.moveBackward(120);
    delay(1500);
    motors.setLeftSpeed(100);
    motors.setRightSpeed(180);
    delay(1000);
    motors.moveForward(160);
  }
  if (backRawValue < LINE_THRESHOLD) {
    motors.moveForward(160);}
}
void loop() {
  main();
  motors.stopMotors();
}
