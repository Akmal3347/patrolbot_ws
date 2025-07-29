/*********************************************************************
 * Arduino firmware for differential drive with mixed protocols:
 *
 * Supported commands (terminated by \r or \n):
 *   - "R<spd>,L<spd>"     : left/right speeds in [-1.0, 1.0] (your existing format)
 *   - "e"                 : reply "<left_enc> <right_enc>\r\n"  (for ArduinoComms.read_encoder_values)
 *   - "m <L> <R>"         : target encoder counts per control loop (for ros2_control)
 *   - "u P:D:I:O"         : update PID gains (optional)
 *
 * Encoder ticks are periodically streamed as:
 *   "L:<ticks> R:<ticks>\r\n"
 *
 * NOTE: Pins 18 & 19 => Arduino Mega. If you use an Uno, remap.
 * NOTE: Only ONE PWM pin ("speedMotor") is provided, so both motors share speed.
 *       For true differential control, give each motor its own PWM pin.
 ********************************************************************/

// ---------------- Pin definitions ----------------
#define alarm        7
#define red_light    6
#define green_light  5

#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9

// Single PWM for both motors (limitation!)
#define speedMotor 10

// Encoders (Arduino Mega pins)
#define leftEncoderPin  18
#define rightEncoderPin 19

// ---------------- Config ----------------
const unsigned long sendInterval = 100;   // ms: periodic encoder publish
const uint16_t LOOP_RATE_HZ = 50;         // must match cfg_.loop_rate (ros2_control)
const uint16_t LOOP_PERIOD_MS = 1000 / LOOP_RATE_HZ;
const long ENC_COUNTS_PER_REV = 980;      // must match cfg_.enc_counts_per_rev

// PWM limits
const int pwmMax = 255;

// ---------------- State ----------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

long lastSentLeftTicks = 0;
long lastSentRightTicks = 0;

unsigned long lastSendTime = 0;
unsigned long lastLoopTime = 0;
unsigned long last_cmd_time_ms = 0;

float leftSpeedCmd = 0.0f;   // [-1, 1] (your legacy interface)
float rightSpeedCmd = 0.0f;  // [-1, 1]

const unsigned long CMD_TIMEOUT_MS = 1000;

// For ros2_control PID "m <counts_per_loop>" path
struct PID {
  int kp = 20, kd = 0, ki = 0, ko = 0;
  long err_sum = 0;
  long last_err = 0;
};
PID pid_left, pid_right;

volatile long target_counts_left = 0;
volatile long target_counts_right = 0;

long prev_enc_left = 0;
long prev_enc_right = 0;

// Serial
const uint16_t SERIAL_TIMEOUT_MS = 100;
String rx;

// ---------------- Prototypes ----------------
void parseSerialCommand();
void updateMotors_FromNormalizedSpeeds(); // legacy R/L path (shared PWM)
void publishEncoderCounts();
void countLeft();
void countRight();
int pidStep(PID &pid, long target_counts, long measured_counts);
void applyPIDLoop();

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);
  analogWrite(speedMotor, 0);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),  countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);

  lastLoopTime = millis();
  lastSendTime = millis();
  last_cmd_time_ms = millis();

  Serial.print("READY\r\n");
}

// ---------------- Main loop ----------------
void loop() {
  // 1) Process any incoming serial cmd (both protocols supported)
  parseSerialCommand();

  // 2) Control loop (50 Hz)
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_PERIOD_MS) {
    lastLoopTime = now;

    // If no ROS "m" command for a while, zero targets (safety)
    if ((now - last_cmd_time_ms) > CMD_TIMEOUT_MS) {
      target_counts_left = 0;
      target_counts_right = 0;
    }

    // PID loop for ros2_control counts-per-loop interface
    applyPIDLoop();

    // If you want to prioritize your old "R,L" velocity path instead,
    // comment out applyPIDLoop() above and uncomment this:
    // updateMotors_FromNormalizedSpeeds();
  }

  // 3) Periodically stream encoder ticks (legacy format)
  if (now - lastSendTime >= sendInterval) {
    publishEncoderCounts();
    lastSendTime = now;
  }
}

// ---------------- Serial handling ----------------
void parseSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      String line = rx;
      rx = "";
      line.trim();
      if (line.length() == 0) continue;

      // ---- "e" -> print "left right"
      if (line == "e") {
        Serial.print(leftTicks);
        Serial.print(" ");
        Serial.print(rightTicks);
        Serial.print("\r\n");
        continue;
      }

      // ---- "m <L> <R>" -> set target counts per loop (ros2_control)
      if (line.charAt(0) == 'm') {
        long L, R;
        int n = sscanf(line.c_str(), "m %ld %ld", &L, &R);
        if (n == 2) {
          target_counts_left = L;
          target_counts_right = R;
          last_cmd_time_ms = millis();
          Serial.print("OK\r\n");
        } else {
          Serial.print("ERR m\r\n");
        }
        continue;
      }

      // ---- "u P:D:I:O" -> set PID gains
      if (line.charAt(0) == 'u') {
        int P, D, I, O;
        int n = sscanf(line.c_str(), "u %d:%d:%d:%d", &P, &D, &I, &O);
        if (n == 4) {
          pid_left.kp = pid_right.kp = P;
          pid_left.kd = pid_right.kd = D;
          pid_left.ki = pid_right.ki = I;
          pid_left.ko = pid_right.ko = O;
          Serial.print("OK\r\n");
        } else {
          Serial.print("ERR u\r\n");
        }
        continue;
      }

      // ---- Legacy: "R<spd>,L<spd>"
      {
        int rIndex = line.indexOf('R');
        int lIndex = line.indexOf('L');
        if (rIndex != -1 && lIndex != -1) {
          float rSpeed = 0.0f, lSpeed = 0.0f;

          int commaIndex = line.indexOf(',', rIndex);
          if (commaIndex != -1) {
            String rStr = line.substring(rIndex + 1, commaIndex);
            rSpeed = rStr.toFloat();
          }

          String lStr = line.substring(lIndex + 1);
          lSpeed = lStr.toFloat();

          rSpeed = constrain(rSpeed, -1.0f, 1.0f);
          lSpeed = constrain(lSpeed, -1.0f, 1.0f);

          leftSpeedCmd = lSpeed;
          rightSpeedCmd = rSpeed;
          last_cmd_time_ms = millis();

          // (Optional) immediately apply the legacy speed command:
          updateMotors_FromNormalizedSpeeds();

          Serial.print("OK\r\n");
          continue;
        }
      }

      Serial.print("ERR ?\r\n");
    } else {
      rx += c;
      if (rx.length() > 128) {
        rx = "";
      }
    }
  }
}

// ---------------- Motor control (legacy path) ----------------
void updateMotors_FromNormalizedSpeeds() {
  // Because you only have ONE PWM pin for both motors,
  // we drive BOTH with the SAME magnitude (worst of the two).
  // You still get direction per motor, but no independent magnitude control.
  // For true differential drive, give each motor its own PWM pin
  // (e.g., speedMotorLeft, speedMotorRight).
  float magL = fabs(leftSpeedCmd);
  float magR = fabs(rightSpeedCmd);
  float mag = max(magL, magR);
  int pwm = (int)(mag * pwmMax);
  if (pwm > 255) pwm = 255;

  // Left direction
  if (leftSpeedCmd >= 0) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  } else {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  }

  // Right direction
  if (rightSpeedCmd >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }

  analogWrite(speedMotor, pwm);
}

// ---------------- PID loop for ros2_control ----------------
void applyPIDLoop() {
  long cur_left = leftTicks;
  long cur_right = rightTicks;

  long delta_left = cur_left - prev_enc_left;
  long delta_right = cur_right - prev_enc_right;

  prev_enc_left = cur_left;
  prev_enc_right = cur_right;

  int pwm_left = pidStep(pid_left, target_counts_left, delta_left);
  int pwm_right = pidStep(pid_right, target_counts_right, delta_right);

  // With shared PWM, we can't apply pwm_left/pwm_right separately.
  // We'll approximate by using the max magnitude and set directions independently.
  int pwm_mag = max(abs(pwm_left), abs(pwm_right));
  if (pwm_mag > 255) pwm_mag = 255;

  // Left direction from PID
  if (pwm_left >= 0) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  } else {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  }

  // Right direction from PID
  if (pwm_right >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  }

  analogWrite(speedMotor, pwm_mag);
}

int pidStep(PID &pid, long target_counts, long measured_counts) {
  long err = target_counts - measured_counts;
  pid.err_sum += err;
  long d_err = err - pid.last_err;
  pid.last_err = err;

  long out = (long)pid.kp * err +
             (long)pid.ki * pid.err_sum +
             (long)pid.kd * d_err;

  if (out > 255) out = 255;
  if (out < -255) out = -255;

  return (int)out;
}

// ---------------- Encoders ----------------
void countLeft() {
  leftTicks++;
}

void countRight() {
  rightTicks++;
}

// ---------------- Telemetry ----------------
void publishEncoderCounts() {
  if (leftTicks != lastSentLeftTicks || rightTicks != lastSentRightTicks) {
    Serial.print("L:");
    Serial.print(leftTicks);
    Serial.print(" R:");
    Serial.println(rightTicks);

    lastSentLeftTicks = leftTicks;
    lastSentRightTicks = rightTicks;
  }
}
