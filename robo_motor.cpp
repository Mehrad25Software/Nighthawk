/ Motor control pins (adjust to your wiring)

//need inputs
//const int IN1 = ***;  // Motor driver input 1
//const int IN2 = ***;  // Motor driver input 2
//const int ENA = ***;  // PWM speed control 

// Speed variable (0-255 for 8-bit PWM)
int motorSpeed = 200;  // Example speed

void setup() {
  Serial.begin(115200);
  Serial.println("Motor setup starting...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  ledcAttachPin(ENA, 0);
  ledcSetup(0, 2000, 8);

  Serial.println("Motor ready.");

  // Start with motor stopped
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // Attach ENA to PWM channel 0
  ledcAttachPin(ENA, 0);  
  ledcSetup(0, 2000, 8); 
  ledcWrite(0, 0);
}

//IN1 --> HIGH, IN2 --> LOW: Move forward
//IN1 --> LOW, IN2 --> HIGH: Move backward
//IN1 --> LOW, IN2 --> LOW: Stops
//IN1 --> HIGH, IN2 --> HIGH: Brakes


void loop() {
  // Move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //let set speed
  ledcWrite(0, motorSpeed);  

  //run forward for 5 seconds
  delay(5000); 

  // Move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(0, motorSpeed);  
 // Run backward for 5 seconds
  delay(5000); 

  // Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(0, 0);
// Stop for 5 seconds
  delay(5000);  

  // Brakes 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(0, 0);
//Waits for 5 seconds
  delay(5000);




  
}
