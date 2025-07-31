// Existing motor and sensor pins
#define alarm 7
#define red_light 6
#define green_light 5

#define motorA1 4      //change from 2 to 4
#define motorA2 12     //change from 3 to 12
#define motorB1 8
#define motorB2 9

#define leftPWM 10         // Left motor PWM (must be PWM pin)
#define rightPWM 11        // Right motor PWM (must be PWM pin)

const int trigPinM = A1;
const int echoPinM = A0;
const int trigPinR = A3;
const int echoPinR = A2;
const int trigPinL = A4;
const int echoPinL = A5;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

#define leftEncoderPin 2  // Change from 18 to 2
#define rightEncoderPin 3 //change from 19 to 3

long duration;
int distance;
int distM, distR, distL;

int threshold_dist = 50;

int leftSpeed = 125;//add leftSpeed(0-255)
int rightSpeed = 125;//add rightSpeed(0-255)

void setup() {
  Serial.begin(9600);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(leftPWM, OUTPUT); pinMode(rightPWM, OUTPUT);  // add PWM pins
  analogWrite(leftPWM, leftSpeed); // add left motor speed
  analogWrite(rightPWM, rightSpeed); // add right motor speed

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);

  delay(5000);
}

void loop() {
  distM = ultrasonic(trigPinM, echoPinM);
  distR = ultrasonic(trigPinR, echoPinR);
  distL = ultrasonic(trigPinL, echoPinL);

  if (distM < threshold_dist && distR < threshold_dist - 20) {
    Rmotor_forward();
    Lmotor_reverse();
  } else if (distM < threshold_dist && distL < threshold_dist - 20) {
    Lmotor_forward();
    Rmotor_reverse();
  } else if (distM < threshold_dist) {
    Rmotor_stop(); Lmotor_stop(); Rlight_on(); Glight_off(); //alarm_on();
  } else {
    Rmotor_forward(); Lmotor_forward(); Glight_on(); Rlight_off(); alarm_off();
  }

  // Send encoder ticks to Raspberry Pi
  Serial.print("ENC_L:"); Serial.print(leftTicks);
  Serial.print(" ENC_R:"); Serial.println(rightTicks);

  delay(100); // Limit the update rate
}

void countLeft() { leftTicks++; }
void countRight() { rightTicks++; }

int ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

// Existing motor and light control functions below here…//
void Rmotor_forward()  { digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW); analogWrite(rightPWM, rightSpeed); }//add rightPWM
void Rmotor_reverse()  { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, HIGH); analogWrite(rightPWM, rightSpeed); }//add rightPWM
void Rmotor_stop()     { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, LOW); analogWrite(rightPWM, 0); }//add rightPWM

void Lmotor_forward()  { digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW);  analogWrite(leftPWM, leftSpeed);}//add leftPWM
void Lmotor_reverse()  { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, HIGH); analogWrite(leftPWM, leftSpeed);}//add leftPWM
void Lmotor_stop()     { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, LOW); analogWrite(leftPWM, 0); }//add leftPWM

//Light and Alarm control functions
void Glight_on()       { digitalWrite(green_light, HIGH); }
void Glight_off()      { digitalWrite(green_light, LOW); }
void Rlight_on()       { digitalWrite(red_light, HIGH); }
void Rlight_off()      { digitalWrite(red_light, LOW); }
void alarm_on()        { digitalWrite(alarm, HIGH); }
void alarm_off()       { digitalWrite(alarm, LOW); }