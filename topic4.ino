#include <Wire.h>
#define MOTOR_ADDR 0x2A
#define LINE_ADDR 0x09
#define NUM_SENSORS 8
/* ================= ULTRA-TUNED RACING PARAMETERS ================= */
// Speed limits (optimized for stability + speed)
#define BASE_SPEED 70 // Increased from 52 (but stable)
#define MAX_SPEED 85  // Increased from 75
#define MIN_SPEED -85 // Symmetric
// SMOOTH & FAST PID (eliminates wiggling)
#define KP_STRAIGHT 3.8f  // REDUCED from 5.0 (less aggressive = smoother)
#define KI_STRAIGHT 0.15f // NEW: Integral term (eliminates drift)
#define KD_STRAIGHT 12.0f // INCREASED from 5.5 (more damping = less oscillation)
#define KP_CURVE 8.275f  // REDUCED from 10.0 (smoother cornering)
#define KI_CURVE 0.08f // Smaller integral for curves
#define KD_CURVE 13.00f  // INCREASED from 3.0 (prevent overshoot)
// Lost-line recovery (slightly reduced for stability)
#define KP_MULTIPLIER 2.8f // Reduced from 3.0
#define KD_MULTIPLIER 2.3f // Increased from 1.5 (smoother recovery)
// Thresholds
#define BLACK_THRESHOLD 2200
#define CURVE_ANTICIPATION_TH 16 // Reduced from 15 (less aggressive)
// Compensation
#define RIGHT_COMPENSATION 3 // Reduced from 5 (smoother)
#define LEFT_COMPENSATION 3  // Reduced from 5
#define PIVOT_TIMEOUT 700 // Reduced from 1200
// ADAPTIVE ACCELERATION (key to smooth + fast)
#define MAX_ACCEL_STRAIGHT 21 // Fast on straights
#define MAX_ACCEL_CURVE 9     // Smooth on curves
#define MAX_ACCEL_LOST 16     // Medium on recovery
// FILTERING CONSTANTS (eliminates wiggle)
#define ERROR_FILTER_ALPHA 0.445f // Error smoothing (0.3-0.5 optimal)
#define DERIV_FILTER_ALPHA 0.525f // Derivative low-pass (critical!)
#define PREDICTIVE_GAIN 0.425f    // REDUCED from 0.8 (less overshoot)
// Integral anti-windup
#define I_MAX 150.0f // Integral clamp
/* ================= CALIBRATION DATA ================= */
const unsigned int sensorWCal[NUM_SENSORS] = {17409, 20251, 21762, 19201,
                                              20226, 21734, 20481, 16130};
const unsigned int sensorBCal[NUM_SENSORS] = {2817, 3584, 3585, 3074,
                                              3328, 3075, 3072, 2306};
const int8_t POSITION[NUM_SENSORS] = {-35, -25, -15, -5, 5, 15, 25, 35};
/* ================= STATE VARIABLES ================= */
unsigned int sensorRaw[NUM_SENSORS];
int sensorNorm[NUM_SENSORS];
// PID state
float error = 0.0f; // Now float for smooth filtering
float lastError = 0.0f;
float errorFiltered = 0.0f; // NEW: Filtered error
float integral = 0.0f;      // NEW: Integral term
float derivative = 0.0f;    // For display
float derivFiltered = 0.0f; // NEW: Filtered derivative
int lastValidError = 0;
int errorVelocity = 0;
// Motor control
int16_t currentLeftSpeed = 0;
int16_t currentRightSpeed = 0;
int16_t lastSentLeft = 0;
int16_t lastSentRight = 0;
// State flags
unsigned long lostLineStartTime = 0;
bool is_line_lost = false;
bool is_soft_curve = false;
/* ================= OPTIMIZED MOTOR CONTROL ================= */
inline void writeU16(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}
void stopAll() {
  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('h');
  Wire.write('a');
  Wire.endTransmission();
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  lastSentLeft = 0;
  lastSentRight = 0;
  integral = 0.0f; // Reset integral on stop
}
// Adaptive acceleration limiting
void setMotorsLR(int targetL, int targetR) {
  // ADAPTIVE ACCEL: faster on straights, smoother on curves
  int maxAccel;
  if (is_line_lost) {
    maxAccel = MAX_ACCEL_LOST;
  } else if (is_soft_curve || abs(error) > 10) {
    maxAccel = MAX_ACCEL_CURVE; // Smooth on curves
  } else {
    maxAccel = MAX_ACCEL_STRAIGHT; // Fast on straights
  }
  // Apply acceleration limiting
  if (currentLeftSpeed < targetL) {
    currentLeftSpeed = min(currentLeftSpeed + maxAccel, targetL);
  } else if (currentLeftSpeed > targetL) {
    currentLeftSpeed = max(currentLeftSpeed - maxAccel, targetL);
  }
  if (currentRightSpeed < targetR) {
    currentRightSpeed = min(currentRightSpeed + maxAccel, targetR);
  } else if (currentRightSpeed > targetR) {
    currentRightSpeed = max(currentRightSpeed - maxAccel, targetR);
  }
  currentLeftSpeed = constrain(currentLeftSpeed, MIN_SPEED, MAX_SPEED);
  currentRightSpeed = constrain(currentRightSpeed, MIN_SPEED, MAX_SPEED);
  // Smart caching
  if (currentLeftSpeed == lastSentLeft && currentRightSpeed == lastSentRight) {
    return;
  }
  lastSentLeft = currentLeftSpeed;
  lastSentRight = currentRightSpeed;
  // I2C transmission
  char dirL = (currentLeftSpeed >= 0) ? 'f' : 'r';
  char dirR = (currentRightSpeed >= 0) ? 'f' : 'r';
  uint16_t spL = abs(currentLeftSpeed);
  uint16_t spR = abs(currentRightSpeed);
  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('b');
  Wire.write('a');
  Wire.write(dirL);
  Wire.write(dirL);
  Wire.write(dirR);
  Wire.write(dirR);
  writeU16(spL);
  writeU16(spL);
  writeU16(spR);
  writeU16(spR);
  Wire.endTransmission();
}
/* ================= SENSOR READING ================= */
inline void readSensorRawAndNormalize() {
  unsigned char raw[16];
  Wire.requestFrom(LINE_ADDR, (uint8_t)16);
  uint8_t n = 0;
  while (Wire.available() && n < 16) {
    raw[n++] = Wire.read();
  }
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorRaw[i] = ((uint16_t)raw[i << 1] << 8) | raw[(i << 1) + 1];
    unsigned int v = sensorRaw[i];
    if (v < sensorBCal[i])
      v = sensorBCal[i];
    if (v > sensorWCal[i])
      v = sensorWCal[i];
    long range = sensorWCal[i] - sensorBCal[i];
    if (range > 0) {
      long scaled = ((long)(v - sensorBCal[i]) * 6144L) / range;
      sensorNorm[i] = constrain(scaled, 0, 6144);
    } else {
      sensorNorm[i] = 0;
    }
  }
}
/* ================= ADVANCED ERROR COMPUTATION ================= */
float computeError() {
  long weightedSum = 0;
  long weightSum = 0;
  int blackCount = 0;
  int whiteCount = 0;
  bool midBlack = false;
  bool leftEdge = false;
  bool rightEdge = false;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorNorm[i] < BLACK_THRESHOLD) {
      weightedSum += (long)sensorNorm[i] * POSITION[i];
      weightSum += sensorNorm[i];
      blackCount++;
      if (i >= 2 && i <= 5)
        midBlack = true;
      if (i <= 1)
        leftEdge = true;
      if (i >= 6)
        rightEdge = true;
    } else {
      whiteCount++;
    }
  }
  // Edge cases
  if (blackCount == NUM_SENSORS || whiteCount == NUM_SENSORS) {
    return errorFiltered; // Return filtered error
  }
  // Compute raw error
  float currentError = 0.0f;
  if (weightSum == 0) {
    currentError = errorFiltered;
  } else {
    currentError = (float)(weightedSum) / (float)(weightSum);
  }
  // Soft curve detection (optimized threshold)
  is_soft_curve = midBlack && abs(currentError) < CURVE_ANTICIPATION_TH;
  if (is_soft_curve) {
    is_line_lost = false;
    lastValidError = (int)currentError;
    // Apply error smoothing filter (KEY TO ELIMINATING WIGGLE)
    errorFiltered = ERROR_FILTER_ALPHA * currentError +
                    (1.0f - ERROR_FILTER_ALPHA) * errorFiltered;
    return errorFiltered;
  }
  // Lost-line detection
  if (!midBlack && (leftEdge || rightEdge)) {
    if (!is_line_lost) {
      is_line_lost = true;
      lostLineStartTime = millis();
      lastValidError = (int)currentError;
      integral = 0.0f; // Reset integral when lost
    }
    return (float)lastValidError;
  }
  // Normal state - apply smoothing
  is_line_lost = false;
  lastValidError = (int)currentError;
  errorFiltered = ERROR_FILTER_ALPHA * currentError +
                  (1.0f - ERROR_FILTER_ALPHA) * errorFiltered;
  return errorFiltered;
}
/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  stopAll();
  delay(300);
  Serial.println(F("=== ULTRA-OPTIMIZED LINE FOLLOWER ==="));
  Serial.println(F("Target: Sub-30s with ZERO wiggle"));
  Serial.print(F("Base: "));
  Serial.print(BASE_SPEED);
  Serial.print(F(" Max: "));
  Serial.println(MAX_SPEED);
  Serial.println(F("Features: PID+I, filtered D, adaptive accel, 500Hz"));
  Serial.println(F("GO!"));
}
/* ================= MAIN LOOP (500Hz ULTRA-FAST) ================= */
void loop() {
  readSensorRawAndNormalize();
  // Get filtered error
  float currentError = computeError();
  int targetLeftSpeed, targetRightSpeed;
  float KP, KI, KD;
  // ===== GAIN SELECTION WITH INTEGRAL =====
  if (is_line_lost) {
    KP = (abs(lastValidError) > 10) ? KP_CURVE * KP_MULTIPLIER
                                    : KP_STRAIGHT * KP_MULTIPLIER;
    KI = 0.0f; // Disable integral when lost
    KD = (abs(lastValidError) > 10) ? KD_CURVE * KD_MULTIPLIER
                                    : KD_STRAIGHT * KD_MULTIPLIER;
    if (millis() - lostLineStartTime > PIVOT_TIMEOUT) {
      is_line_lost = false;
      lastError = 0.0f;
      integral = 0.0f;
    }
  } else {
    if (is_soft_curve) {
      KP = KP_STRAIGHT;
      KI = KI_STRAIGHT;
      KD = KD_STRAIGHT;
    } else {
      KP = (abs(currentError) > 10) ? KP_CURVE : KP_STRAIGHT;
      KI = (abs(currentError) > 10) ? KI_CURVE : KI_STRAIGHT;
      KD = (abs(currentError) > 10) ? KD_CURVE : KD_STRAIGHT;
    }
  }
  // ===== ADVANCED PID WITH FILTERED DERIVATIVE =====
  // Integral term with anti-windup
  if (!is_line_lost) {
    integral += currentError;
    integral = constrain(integral, -I_MAX, I_MAX);
  }
  // Raw derivative
  float rawDerivative = currentError - lastError;
  // LOW-PASS FILTER THE DERIVATIVE (CRITICAL FOR SMOOTHNESS!)
  derivFiltered = DERIV_FILTER_ALPHA * rawDerivative +
                  (1.0f - DERIV_FILTER_ALPHA) * derivFiltered;
  derivative = derivFiltered; // Store for display
  // Reduced predictive term (prevents overshoot)
  float predictiveError = currentError + (rawDerivative * PREDICTIVE_GAIN);
  // Full PID calculation
  float correction = KP * predictiveError + KI * integral + KD * derivFiltered;
  correction = constrain(correction, (float)-MAX_SPEED, (float)MAX_SPEED);
  // ===== MINIMAL COMPENSATION =====
  int leftComp = (currentError < 0) ? LEFT_COMPENSATION : 0;
  int rightComp = (currentError > 0) ? RIGHT_COMPENSATION : 0;
  targetLeftSpeed = BASE_SPEED - (int)correction + leftComp;
  targetRightSpeed = BASE_SPEED + (int)correction + rightComp;
  targetLeftSpeed = constrain(targetLeftSpeed, MIN_SPEED, MAX_SPEED);
  targetRightSpeed = constrain(targetRightSpeed, MIN_SPEED, MAX_SPEED);
  setMotorsLR(targetLeftSpeed, targetRightSpeed);
  lastError = currentError;
  // ===== ULTRA-FAST LOOP: 2ms = 500Hz =====
  delay(2);
}