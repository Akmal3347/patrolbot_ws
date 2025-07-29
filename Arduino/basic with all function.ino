// Motor & Lights
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9
#define speedMotor 10

// Ultrasonic Pins
const int trigPinM = A1;
const int echoPinM = A0;
const int trigPinR = A3;
const int echoPinR = A2;
const int trigPinL = A4;
const int echoPinL = A5;

// Encoder Pins
#define leftEncoderPin 18 // not set yet
#define rightEncoderPin 19 //not set yet
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// State Variables
int threshold_dist = 50;
int distM, distR, distL;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(50);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);
  analogWrite(speedMotor, 125);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);
}

void loop() {
  readUltrasonics();

  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  sendEncoderData();
  delay(100);  // limit loop speed
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processSerialCommand(String cmd) {
  cmd.trim();  // Remove trailing newline and whitespace

  // ROS 2 motor driver node sends single characters
  if (cmd.length() == 1) {
    char c = cmd.charAt(0);
    switch (c) {
      case 'f':
        Lmotor_forward(); Rmotor_forward();
        break;
      case 'b':
        Lmotor_reverse(); Rmotor_reverse();
        break;
      case 'l':
        Lmotor_reverse(); Rmotor_forward();
        break;
      case 'r':
        Lmotor_forward(); Rmotor_reverse();
        break;
      case 's':
        Lmotor_stop(); Rmotor_stop();
        break;
      default:
        break;
    }
  }
  // Optional: support CMD:<left>:<right> format (can be used later)
  else if (cmd.startsWith("CMD:")) {
    int sep = cmd.indexOf(':', 4);
    if (sep > 4) {
      int leftDir = cmd.substring(4, sep).toInt();
      int rightDir = cmd.substring(sep + 1).toInt();
      setMotorDirection(leftDir, rightDir);
    }
  }
}


void setMotorDirection(int leftDir, int rightDir) {
  // Left motor
  if (leftDir > 0) { Lmotor_forward(); }
  else if (leftDir < 0) { Lmotor_reverse(); }
  else { Lmotor_stop(); }

  // Right motor
  if (rightDir > 0) { Rmotor_forward(); }
  else if (rightDir < 0) { Rmotor_reverse(); }
  else { Rmotor_stop(); }
}

void readUltrasonics() {
  distM = ultrasonic(trigPinM, echoPinM);
  distR = ultrasonic(trigPinR, echoPinR);
  distL = ultrasonic(trigPinL, echoPinL);

  if (distM < threshold_dist && distR < threshold_dist - 20) {
    Rmotor_forward(); Lmotor_reverse();
  } else if (distM < threshold_dist && distL < threshold_dist - 20) {
    Lmotor_forward(); Rmotor_reverse();
  } else if (distM < threshold_dist) {
    Rmotor_stop(); Lmotor_stop(); Rlight_on(); Glight_off();
  } else {
    Glight_on(); Rlight_off(); alarm_off();
  }
}

void sendEncoderData() {
  Serial.print("ENC_L:"); Serial.print(leftTicks);
  Serial.print(" ENC_R:"); Serial.println(rightTicks);
}

int ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// Motor control functions
void Rmotor_forward()  { digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW); }
void Rmotor_reverse()  { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, HIGH); }
void Rmotor_stop()     { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, LOW); }

void Lmotor_forward()  { digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW); }
void Lmotor_reverse()  { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, HIGH); }
void Lmotor_stop()     { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, LOW); }

void Glight_on()       { digitalWrite(green_light, HIGH); }
void Glight_off()      { digitalWrite(green_light, LOW); }
void Rlight_on()       { digitalWrite(red_light, HIGH); }
void Rlight_off()      { digitalWrite(red_light, LOW); }
void alarm_on()        { digitalWrite(alarm, HIGH); }
void alarm_off()       { digitalWrite(alarm, LOW); }

void countLeft() { leftTicks++; }
void countRight() { rightTicks++; }
