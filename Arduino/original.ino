#define alarm 7
#define red_light 6
#define green_light 5

#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9

#define speedMotor 10

const int trigPinM = A1;
const int echoPinM = A0;

const int trigPinR = A3;
const int echoPinR = A2;

const int trigPinL = A4;
const int echoPinL = A5;

long duration;
int distance;
int distM, distR, distL;

//Must be more than 25
int threshold_dist = 50;

void setup()
{
  Serial.begin(9600); 

  pinMode(alarm,OUTPUT); 
  pinMode(red_light,OUTPUT); 
  pinMode(green_light,OUTPUT); 
   
  pinMode(trigPinM,OUTPUT); 
  pinMode(echoPinM,INPUT); 
  pinMode(trigPinR,OUTPUT); 
  pinMode(echoPinR,INPUT); 
  pinMode(trigPinL,OUTPUT); 
  pinMode(echoPinL,INPUT); 
  
  pinMode(motorA1,OUTPUT);
  pinMode(motorA2,OUTPUT);
  pinMode(motorB1,OUTPUT);
  pinMode(motorB2,OUTPUT);

  pinMode(speedMotor,OUTPUT);
  analogWrite(speedMotor,125);

  delay(5000);
}

void loop()
{
  distM =ultrasonic(trigPinM, echoPinM);
  Serial.print("Distance [M]: ");
  Serial.println(distM);
  delay(10);

  distR =ultrasonic(trigPinR, echoPinR);
  Serial.print("Distance [R]: ");
  Serial.println(distR);
  delay(10);

  distL =ultrasonic(trigPinL, echoPinL);
  Serial.print("Distance [L]: ");
  Serial.println(distL);
  delay(10);

  
  if(distM < threshold_dist && distR < threshold_dist-20)
  {
    distM =ultrasonic(trigPinM, echoPinM);
    delay(10);

    distR =ultrasonic(trigPinR, echoPinR);
    delay(10);
  
    Rmotor_forward();
    Lmotor_reverse();
  }
  else if(distM < threshold_dist && distL < threshold_dist-20)
  {
    distM =ultrasonic(trigPinM, echoPinM);
    delay(10);

    distR =ultrasonic(trigPinR, echoPinR);
    delay(10);
  
    Lmotor_forward();
    Rmotor_reverse();
  }
  else if(distM < threshold_dist)
  {
    distM =ultrasonic(trigPinM, echoPinM);
    delay(10);

    Rmotor_stop();
    Lmotor_stop();
    Rlight_on();
    Glight_off();
    //alarm_on();
  }
  else
  {
    Rmotor_forward();
    Lmotor_forward();
    Glight_on();
    Rlight_off();
    alarm_off(); 
  }
  

}

void Rlight_on()
{
  digitalWrite(red_light,HIGH);
}

void Rlight_off()
{
  digitalWrite(red_light,LOW);
}

void Glight_on()
{
  digitalWrite(green_light,HIGH);
}

void Glight_off()
{
  digitalWrite(green_light,LOW);
}

void alarm_on()
{
  digitalWrite(alarm,HIGH);
}

void alarm_off()
{
  digitalWrite(alarm,LOW);
}

void Rmotor_forward()
{
  digitalWrite(motorA1,HIGH);
  digitalWrite(motorA2,LOW);
}

void Rmotor_reverse()
{
  digitalWrite(motorA1,LOW);
  digitalWrite(motorA2,HIGH);
}

void Rmotor_stop()
{
  digitalWrite(motorA1,LOW);
  digitalWrite(motorA2,LOW);
}

void Lmotor_forward()
{
  digitalWrite(motorB1,HIGH);
  digitalWrite(motorB2,LOW);
}

void Lmotor_reverse()
{
  digitalWrite(motorB1,LOW);
  digitalWrite(motorB2,HIGH);
}

void Lmotor_stop()
{
  digitalWrite(motorB1,LOW);
  digitalWrite(motorB2,LOW);
}

int ultrasonic(int trigPin, int echoPin)
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);

  return distance;
}
