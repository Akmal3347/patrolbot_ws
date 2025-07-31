// Pin definitions// code for motors based on serial input only
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 4 // change from 2 to 4
#define motorA2 12 // change from 3 to 12
#define motorB1 8
#define motorB2 9
#define leftPWM 10 // PWM pin for left motor
#define rightPWM 11 // PWM pin for right motor

#define leftEncoderPin 2 // change from 18 to 2
#define rightEncoderPin 3 // change from 19 to 3

// Encoder tick counts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

long lastSentLeftTicks = 0;
long lastSentRightTicks = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // Send encoder data every 100ms

// Motor speed variables
float leftSpeedCmd = 0.0;  // Range: -1.0 to 1.0
float rightSpeedCmd = 0.0; // Range: -1.0 to 1.0

// PWM limits
const int pwmMax = 255;

// Function prototypes
void parseSerialCommand();
void updateMotors();
void sendEncoderCounts();

void setup() {
  Serial.begin(115200);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(leftPWM, OUTPUT); pinMode(rightPWM, OUTPUT);
  analogWrite(leftPWM, 200); analogWrite(rightPWM, 200) // change from 255 to 200 for initial speed

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);
}

void loop() {
  // Parse incoming serial commands for motor control
  parseSerialCommand();

  // Update motor PWM signals based on commands
  updateMotors();

  // Periodically send encoder counts for odometry
  unsigned long now = millis();
  if (now - lastSendTime >= sendInterval) {
    sendEncoderCounts();
    lastSendTime = now;
  }
}

// Parse serial input for motor commands
void parseSerialCommand() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      continue;

    // Expected format: "R<speed>,L<speed>"
    int rIndex = line.indexOf('R');
    int lIndex = line.indexOf('L');

    if (rIndex != -1 && lIndex != -1) {
      float rSpeed = 0.0, lSpeed = 0.0;

      // Parse right speed
      int commaIndex = line.indexOf(',', rIndex);
      if (commaIndex != -1) {
        String rStr = line.substring(rIndex + 1, commaIndex);
        rSpeed = rStr.toFloat();
      }

      // Parse left speed
      int endIdx = line.length();
      String lStr = line.substring(lIndex + 1, endIdx);
      lSpeed = lStr.toFloat();

      // Clamp speeds to -1.0 to 1.0
      rSpeed = constrain(rSpeed, -1.0, 1.0);
      lSpeed = constrain(lSpeed, -1.0, 1.0);

      // Update motor commands
      leftSpeedCmd = lSpeed;
      rightSpeedCmd = rSpeed;
    }
  }
}

// Update motor PWM signals based on command speeds
void updateMotors() {
  // Left motor
  if (leftSpeedCmd >= 0) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  } else {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  }
  analogWrite(leftPWM, (int)(abs(leftSpeedCmd) * pwmMax));  // Ensure PWM is within range

  // Right motor
  if (rightSpeedCmd >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  analogWrite(rightPWM, (int)(abs(rightSpeedCmd) * pwmMax));  // Ensure PWM is within range
}

// Interrupt routines for encoder
void countLeft() {
  leftTicks++;
}

void countRight() {
  rightTicks++;
}

// Send encoder counts over serial
void sendEncoderCounts() {
  Serial.print("L:");
  Serial.print(leftTicks);
  Serial.print(" R:");
  Serial.println(rightTicks);
}