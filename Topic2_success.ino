#include <Wire.h>

// ============= Sensor pins (your current) =============
const uint8_t PIN_L = A6;   // left sensor
const uint8_t PIN_R = A7;   // right sensor

// ============= Threshold (you said) =============
const int TH = 40;         // black > 100, white < 100

// ============= Speed (0-100) =============
int speed = 20;             // base speed
int minSpeed = 25;          // IMPORTANT for rear wheels

// ============= PID =============
float Kp = 0.80;
float Ki = 0.06;
float Kd = 0.20;

float error = 0, lastError = 0;
float integral = 0, derivative = 0;
float pid_output = 0;

// ============= I2C safety =============
const uint8_t MOTOR_ADDR_DEC = 42; // your old protocol uses 42 (0x2A)

// ============= Line state machine =============
enum Mode { NORMAL, LOST };
Mode mode = NORMAL;

int lastDir = +1;           // +1 means turn right to find line, -1 turn left
int whiteCount = 0;
const int WHITE_CONFIRM = 3; // how many loops of full-white => LOST

// seek (找线) parameters
const int SEEK_BASE = 25;   // seek base speed (0-100), must be >= minSpeed
const int SEEK_DIFF = 25;   // seek turn strength (bigger for wide sensors)

// stability
const float deadband = 0.02;
const float integralLimit = 2.0;
const float outLimit = 1.2;
const int loopDelayMs = 25;

static inline bool isBlack(int v){ return v > TH; }
static inline bool isWhite(int v){ return v < TH; }

void car_control1(float a, float b) {
  Wire.beginTransmission(MOTOR_ADDR_DEC);
  Wire.write("sa");

  Wire.write((int)b); Wire.write(0); // right front
  Wire.write((int)b); Wire.write(0); // right rear
  Wire.write((int)a); Wire.write(0); // left front
  Wire.write((int)a); Wire.write(0); // left rear

  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Wire.end();
    delay(2);
    Wire.begin();
    Wire.setWireTimeout(3000, true);
  }
  delay(1);
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_L, INPUT);
  pinMode(PIN_R, INPUT);

  Wire.begin();
  Wire.setWireTimeout(3000, true);
}

void loop() {
  int Lraw = analogRead(PIN_L);
  int Rraw = analogRead(PIN_R);

  bool Lb = isBlack(Lraw);
  bool Rb = isBlack(Rraw);
  bool Lw = isWhite(Lraw);
  bool Rw = isWhite(Rraw);

  // update lastDir based on which side sees black (important memory!)
  if (Lb && !Rb) lastDir = -1;        // black on left => likely need turn left to stay on it
  else if (!Lb && Rb) lastDir = +1;   // black on right => turn right

  // ===== Mode switching =====
  if (Lw && Rw) whiteCount++;
  else whiteCount = 0;

  if (mode == NORMAL) {
    if (whiteCount >= WHITE_CONFIRM) {
      mode = LOST;            // enter seek mode
      integral *= 0.3;        // dump integral so it doesn't fight seek
    }
  } else { // LOST
    // If any sensor sees black again -> back to NORMAL
    if (Lb || Rb) {
      mode = NORMAL;
      whiteCount = 0;
      integral *= 0.5;        // soften rebound
      lastError = 0;
    }
  }

  // ===== Control =====
  float leftspeed, rightspeed;

  if (mode == LOST) {
    // >>> KEY FIX: full-white DOES NOT mean go straight
    // Seek by turning to lastDir until line appears
    // For wide sensors + right angle, we need strong turn
    int Ls, Rs;
    if (lastDir > 0) {
      // turn right: left faster, right slower
      Ls = SEEK_BASE + SEEK_DIFF;
      Rs = SEEK_BASE - SEEK_DIFF;
    } else {
      // turn left
      Ls = SEEK_BASE - SEEK_DIFF;
      Rs = SEEK_BASE + SEEK_DIFF;
    }

    // keep wheels from stalling
    Ls = constrain(Ls, 0, 100);
    Rs = constrain(Rs, 0, 100);
    if (Ls > 0 && Ls < minSpeed) Ls = minSpeed;
    if (Rs > 0 && Rs < minSpeed) Rs = minSpeed;

    car_control1(Ls, Rs);
    delay(loopDelayMs);
    return; // done this loop
  }

  // NORMAL mode: PID
  error = (float)(Lraw - Rraw) / 1023.0;
  if (fabs(error) < deadband) error = 0;

  // anti-windup
  if (fabs(error) < 0.8) integral += error;
  else integral *= 0.8;
  integral = constrain(integral, -integralLimit, integralLimit);

  derivative = error - lastError;
  pid_output = Kp * error + Ki * integral + Kd * derivative;
  pid_output = constrain(pid_output, -outLimit, outLimit);
  lastError = error;

  float steer = pid_output * 60.0;   // keep your style
  leftspeed  = speed - steer;
  rightspeed = speed + steer;

  leftspeed  = constrain(leftspeed, 0, 100);
  rightspeed = constrain(rightspeed, 0, 100);

  // keep rear wheels alive
  if (leftspeed  > 0 && leftspeed  < minSpeed) leftspeed  = minSpeed;
  if (rightspeed > 0 && rightspeed < minSpeed) rightspeed = minSpeed;

  car_control1(leftspeed, rightspeed);
  delay(loopDelayMs);
}
