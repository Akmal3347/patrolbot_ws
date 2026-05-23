// ========================
// Pin definitions (Uno)
// ========================
#define alarm 7
#define red_light 6
#define green_light 5

#define motorA1 4     // Left motor direction pin 1
#define motorA2 12    // Left motor direction pin 2
#define motorB1 8     // Right motor direction pin 1
#define motorB2 9     // Right motor direction pin 2
#define leftPWM 10    // Left motor PWM (speed)
#define rightPWM 11   // Right motor PWM (speed)

#define leftEncoderPin 2   // Left encoder
#define rightEncoderPin 3  // Right encoder

// ========================
// Encoder tick counts
// ========================
volatile long leftTicks = 0;
volatile long rightTicks = 0;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // Send encoder data every 100ms

// ========================
// Motor speed variables
// ========================
float leftSpeedCmd = 0.0;   // Range: -1.0 to 1.0
float rightSpeedCmd = 0.0;  // Range: -1.0 to 1.0

// ========================
// Calibration factors
// ========================
const float leftCalibration  = 0.953;   // left motor (keep as baseline)
const float rightCalibration = 1.0;   // >1.0 if right motor is slower

// PWM max
const int pwmMax = 255;

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(115200);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // Encoder pins
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);

  Serial.println("READY");
}

// ========================
// Main loop
// ========================
void loop() {
  parseSerialCommand();   // Get wheel commands from ROS
  updateMotors();         // Drive motors

  unsigned long now = millis();
  if (now - lastSendTime >= sendInterval) {
    sendEncoderCounts();  // Send ticks back to ROS
    lastSendTime = now;
  }
}

// ========================
// Parse serial commands like "R0.8,L0.5"
// ========================
void parseSerialCommand() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int rIndex = line.indexOf('R');
    int lIndex = line.indexOf('L');
    if (rIndex != -1 && lIndex != -1) {
      float rSpeed = 0.0, lSpeed = 0.0;

      int commaIndex = line.indexOf(',', rIndex);
      if (commaIndex != -1) {
        rSpeed = line.substring(rIndex + 1, commaIndex).toFloat();
      }
      lSpeed = line.substring(lIndex + 1).toFloat();

      // Clamp -1.0 to 1.0
      rightSpeedCmd = constrain(rSpeed, -1.0, 1.0);
      leftSpeedCmd = constrain(lSpeed, -1.0, 1.0);
    }
  }
}

// ========================
// Update motor directions and PWM
// ========================
void updateMotors() {
  float leftPWMVal  = abs(leftSpeedCmd)  * leftCalibration;
  float rightPWMVal = abs(rightSpeedCmd) * rightCalibration;

  // Constrain after calibration
  leftPWMVal  = constrain(leftPWMVal,  0.0, 1.0);
  rightPWMVal = constrain(rightPWMVal, 0.0, 1.0);

  // Left motor
  if (leftSpeedCmd >= 0) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  } else {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  }
  analogWrite(leftPWM, (int)(leftPWMVal * pwmMax));

  // Right motor
  if (rightSpeedCmd >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }
  analogWrite(rightPWM, (int)(rightPWMVal * pwmMax));
}

// ========================
// Encoder interrupts (direction-aware)
// ========================
void countLeft() {
  if (leftSpeedCmd >= 0) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void countRight() {
  if (rightSpeedCmd >= 0) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

// ========================
// Send ticks to ROS (format: ENC,left,right)
// ========================
void sendEncoderCounts() {
  Serial.print("ENC,");
  Serial.print(leftTicks);
  Serial.print(",");
  Serial.println(rightTicks);
}
