// ----------------------------- Ros 2 +obstacle override[motor will not respond to Ros2 command-only direction change] -----------------------------
// Pin Definitions
// -----------------------------
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 4
#define motorA2 12   // changed from 3 to 12
#define motorB1 8
#define motorB2 9
#define leftPWM 10   // PWM for left motor
#define rightPWM 11  // PWM for right motor

// Ultrasonic sensors
#define trigPinM A1
#define echoPinM A0
#define trigPinL A2
#define echoPinL A3
#define trigPinR A4
#define echoPinR A5

// Encoders
#define leftEncoderPin 2  // changed from 18 to 2
#define rightEncoderPin 3  // changed from 19 to 3

// -----------------------------
// State Variables
// -----------------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

float leftSpeed = 0.0;
float rightSpeed = 0.0;
unsigned long lastCommandTime = 0;
const unsigned long timeoutMs = 1000;

String inputString = "";
bool stringComplete = false;

bool obstacleDetected = false;
unsigned long lastObstacleTime = 0;
const int obstacleHoldTime = 1000;  // ms
// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  inputString.reserve(50);

  // Motor and Lights
  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(leftPWM, OUTPUT); pinMode(rightPWM, OUTPUT);
  analogWrite(leftPWM, 200); analogWrite(rightPWM, 200); // fixed PWM speed

  // Ultrasonic
  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);

  // Encoders
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);
}

// -----------------------------
// Main Loop
// -----------------------------
void loop() {
  unsigned long now = millis();

  // Check serial command
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = now;
  }

  // Obstacle detection
  int distM = getDistance(trigPinM, echoPinM);
  int distL = getDistance(trigPinL, echoPinL);
  int distR = getDistance(trigPinR, echoPinR);

  obstacleDetected = (distM < 20 || distL < 15 || distR < 15);

  // Light and alarm logic
  if (obstacleDetected) {
    digitalWrite(alarm, HIGH);
    digitalWrite(red_light, HIGH);
    digitalWrite(green_light, LOW);
    stopMotors();  // override ROS command
    lastObstacleTime = now;
  } else {
    digitalWrite(alarm, LOW);
    digitalWrite(red_light, LOW);
    digitalWrite(green_light, HIGH);

    // Only resume if no obstacle recently
    if (now - lastObstacleTime > obstacleHoldTime) {
      if (now - lastCommandTime < timeoutMs) {
        applyMotorSpeeds();  // ROS2 speed
      } else {
        stopMotors();  // No command received
      }
    }
  }

  sendEncoderData();
  delay(50);  // Tune as needed
}

// -----------------------------
// Parse ROS2 Velocity Command
// -----------------------------
void parseCommand(String cmd) {
  cmd.trim();
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex > 0 && spaceIndex < cmd.length() - 1) {
    float l = cmd.substring(0, spaceIndex).toFloat();
    float r = cmd.substring(spaceIndex + 1).toFloat();
    leftSpeed = l;
    rightSpeed = r;
  }
}

// -----------------------------
// Apply Speeds to Motors
// -----------------------------
void applyMotorSpeeds() {
  setMotor(leftSpeed, 'L');
  setMotor(rightSpeed, 'R');
}

void setMotor(float speed, char side) {
  int in1, in2, pwmPin;
  if (side == 'L') {
    in1 = motorA1; in2 = motorA2; pwmPin = leftPWM; // add PWM pin for left motor
  } else {
    in1 = motorB1; in2 = motorB2; pwmPin = rightPWM; // add PWM pin for right motor
  }

  if (speed > 0.01) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < -0.01) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// -----------------------------
// Stop Motors
// -----------------------------
void stopMotors() {
  leftSpeed = 0.0;
  rightSpeed = 0.0;
  digitalWrite(motorA1, LOW); digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); digitalWrite(motorB2, LOW);
}

// -----------------------------
// Encoder Output
// -----------------------------
void sendEncoderData() {
  Serial.print(leftTicks);
  Serial.print(" ");
  Serial.println(rightTicks);
}

// -----------------------------
// Encoder ISRs
// -----------------------------
void countLeft()  { leftTicks++; }
void countRight() { rightTicks++; }

// -----------------------------
// Ultrasonic Distance Reading
// -----------------------------
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);  // 20ms timeout
  int distance = duration * 0.034 / 2;
  return distance;
}

// -----------------------------
// Serial Event for ROS Commands
// -----------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
