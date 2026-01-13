#include <Wire.h>

// ================= 1. Hardware and Address Definitions =================
#define MOTOR_ADDR 0x2A    // Motor driver board I2C address
#define LINE_ADDR  0x09    // 8-channel line tracking sensor I2C address
#define NUM_SENSORS 8      // Number of sensors

// ================= 2. Speed PID Parameters (Core of Variable Speed Control) =================
// Speed PID target values (switch after detecting cross black line)
#define SPEED_TARGET_CONST 20  // Initial target speed
#define SPEED_TARGET_ACCEL 30  // Acceleration target after first cross black line
#define SPEED_TARGET_DECEL 15  // Deceleration target after second cross black line
// Speed PID tuning parameters (independent of tracking PID)
#define SPEED_KP 0.8       
#define SPEED_KI 0.1       
#define SPEED_KD 0.2       
// Speed limits
#define MAX_MOTOR_SPEED 35 
#define MIN_MOTOR_SPEED -35
volatile float currentSpeedTarget = SPEED_TARGET_CONST; // Current speed PID target value
float speedError = 0;       // Speed PID error (target - actual feedback, simplified to target value)
float speedIntegral = 0;    // Speed PID integral term
float speedDerivative = 0;  // Speed PID derivative term
float lastSpeedError = 0;   // Previous cycle speed error

// ================= 3. Tracking PID Parameters (Fixed, only for correction) =================
#define TRACK_KP 8.0       // Tracking proportional gain (fixed)
#define TRACK_KD 3.0       // Tracking derivative gain (fixed)
#define TRACK_KI 0.0       // Tracking integral gain (0 to avoid oscillation)
#define RIGHT_COMPENSATION 4 // Right turn compensation

// ================= 4. Threshold and Time Parameters =================
#define CROSS_BLACK_THRESH 990 // Cross black line detection: 8 sensors ≥990 (close to 1000)
#define TRACK_BLACK_THRESH 70  // Vertical black line tracking threshold
#define MIN_SEARCH_TIME 180    // Right angle line loss search time
#define CONTROL_INTERVAL 10    // Control cycle (10ms = 100Hz)
#define CROSS_DELAY 500        // Cross line debounce delay
#define TRACK_STABLE_DELAY 1000// Startup stabilization delay (prevent false trigger)

// ================= 5. System State Variables =================
enum SpeedState { CONSTANT, ACCEL, DECEL };
SpeedState currentSpeedState = CONSTANT;
int crossLineCount = 0;        // Cross black line counter (0/1/2)
unsigned long lastCrossTime = 0;// Last cross line trigger time
unsigned long lastSerialPrint = 0;
bool trackStable = false;      // Tracking stability flag
unsigned long trackStableTime = 0;

// Tracking state variables
unsigned int sensorRaw[NUM_SENSORS];  
int sensorNorm[NUM_SENSORS];          
int trackError = 0;            // Tracking PID error
int lastTrackError = 0;        // Previous cycle tracking error
bool isLineLost = false;       // Line loss flag
int searchDir = 1;             // Line loss search direction
unsigned long searchStartTime = 0;

// ================= 6. Motor Control Functions =================
void writeU16(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}

void stopAll() {
  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('h'); Wire.write('a');
  Wire.endTransmission();
  Serial.println("[MOTOR] Stopped");
}

void setMotorsLR(int valL, int valR) {
  valL = constrain(valL, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  valR = constrain(valR, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  char dirL = (valL >= 0) ? 'f' : 'r'; // 'f' = forward, 'r' = reverse
  char dirR = (valR >= 0) ? 'f' : 'r';
  uint16_t spL = abs(valL);
  uint16_t spR = abs(valR);

  Wire.beginTransmission(MOTOR_ADDR);
  Wire.write('b'); Wire.write('a');
  Wire.write(dirL); Wire.write(dirL);
  Wire.write(dirR); Wire.write(dirR);
  writeU16(spL); writeU16(spL);
  writeU16(spR); writeU16(spR);
  Wire.endTransmission();
}

// ================= 7. Speed PID Calculation (Output current base speed) =================
float calcSpeedPID() {
  // Speed error: target value - actual speed (simplified to target value when no hardware feedback)
  speedError = currentSpeedTarget;
  // Integral term (clamped to avoid integral windup)
  speedIntegral += speedError * (CONTROL_INTERVAL / 1000.0);
  speedIntegral = constrain(speedIntegral, -5, 5);
  // Derivative term
  speedDerivative = (speedError - lastSpeedError) / (CONTROL_INTERVAL / 1000.0);
  // PID output
  float speedOutput = (SPEED_KP * speedError) + (SPEED_KI * speedIntegral) + (SPEED_KD * speedDerivative);
  // Update previous cycle error
  lastSpeedError = speedError;
  return speedOutput;
}

// ================= 8. Sensor Reading and Normalization =================
void readSensor() {
  unsigned char raw[16] = {0};
  int n = 0;
  Wire.requestFrom(LINE_ADDR, 16);
  while (Wire.available() && n < 16) {
    raw[n++] = Wire.read();
  }

  unsigned int sensorWCal[NUM_SENSORS] = {60418, 59905, 60163, 59906, 60417, 57344, 58625, 52993}; // White calibration values
  unsigned int sensorBCal[NUM_SENSORS] = {5379, 5377, 5376, 5120, 5122, 4608, 4866, 4096};         // Black calibration values

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorRaw[i] = (raw[i * 2] << 8) | raw[i * 2 + 1];
    long v = constrain(sensorRaw[i], sensorBCal[i], sensorWCal[i]);
    long scaled = (v - sensorBCal[i]) * 1000L / (sensorWCal[i] - sensorBCal[i]);
    sensorNorm[i] = constrain(1000 - scaled, 0, 1000); // 0=white, 1000=black
  }
}

// ================= 9. Cross Black Line Detection (8 sensors ≥990) =================
bool isCrossLine() {
  int fullBlack = 0;
  Serial.print("Sensor: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorNorm[i]); Serial.print(" ");
    if (sensorNorm[i] >= CROSS_BLACK_THRESH) fullBlack++;
  }
  Serial.print(" | FullBlack: "); Serial.println(fullBlack);
  return (fullBlack == NUM_SENSORS); // Only detect cross line when all 8 sensors are black
}

// ================= 10. Tracking PID Calculation (Output correction value) =================
int calcTrackPID() {
  long weightedSum = 0;
  long weightSum = 0;
  int pos[NUM_SENSORS] = {-35, -25, -15, -5, 5, 15, 25, 35}; // Sensor position weights

  bool midBlack = false, leftEdge = false, rightEdge = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorNorm[i] > TRACK_BLACK_THRESH) {
      weightedSum += (long)sensorNorm[i] * pos[i];
      weightSum += sensorNorm[i];
      if (i >= 2 && i <= 5) midBlack = true;    // Middle sensors detect black
      if (i <= 1) leftEdge = true;              // Left edge sensors detect black
      if (i >= 6) rightEdge = true;             // Right edge sensors detect black
    }
  }

  // Line loss handling
  if (!midBlack && (leftEdge || rightEdge)) {
    if (!isLineLost) {
      isLineLost = true;
      searchStartTime = millis();
      searchDir = rightEdge ? -1 : 1; // Search left if right edge detected, right if left edge detected
      Serial.println("[TRACK] Lost → Search");
    }
    return lastTrackError; // Maintain last correction value during search
  }

  isLineLost = false;
  trackError = (weightSum == 0) ? 0 : (weightedSum / weightSum);
  // Tracking PID calculation (P+D only)
  float trackOutput = (trackError * TRACK_KP) + ((trackError - lastTrackError) * TRACK_KD);
  lastTrackError = trackError;
  return (int)trackOutput;
}

// ================= 11. Initialization Function =================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  stopAll();
  delay(500);

  // Hardware detection
  bool sensorOK = false, motorOK = false;
  Wire.requestFrom(LINE_ADDR, 1);
  if (Wire.available() > 0) { sensorOK = true; Wire.read(); }
  Wire.requestFrom(MOTOR_ADDR, 1);
  if (Wire.available() > 0) { motorOK = true; Wire.read(); }

  if (!sensorOK || !motorOK) {
    Serial.println(!sensorOK ? "Sensor Error" : "Motor Error");
    while (1); // Halt on hardware error
  }

  Serial.println("[SYSTEM] Ready → CONSTANT Speed");
  trackStableTime = millis();
  lastSerialPrint = millis();
}

// ================= 12. Main Loop (Dual PID Core Logic) =================
void loop() {
  unsigned long now = millis();
  readSensor(); // Read sensor values

  // 1. Startup stability check (cross line detection enabled after 1 second)
  if (!trackStable && (now - trackStableTime >= TRACK_STABLE_DELAY)) {
    trackStable = true;
    Serial.println("[TRACK] Stable → Cross Line Detect ON");
  }

  // 2. Cross line detection + Speed PID target switching
  if (!isLineLost && trackStable) {
    bool crossDetected = isCrossLine();
    if (crossDetected && (now - lastCrossTime >= CROSS_DELAY) && (crossLineCount < 2)) {
      crossLineCount++;
      lastCrossTime = now;
      // Switch speed PID target (core variable speed logic)
      switch (crossLineCount) {
        case 1:
          currentSpeedTarget = SPEED_TARGET_ACCEL;
          currentSpeedState = ACCEL;
          Serial.println("\n[!] 1st Cross → ACCEL (Target: 30)");
          break;
        case 2:
          currentSpeedTarget = SPEED_TARGET_DECEL;
          currentSpeedState = DECEL;
          Serial.println("\n[!] 2nd Cross → DECEL (Target: 15)");
          break;
      }
    }
  }

  // 3. Dual PID calculation (100Hz control)
  static unsigned long lastControl = 0;
  if (now - lastControl >= CONTROL_INTERVAL) {
    lastControl = now;
    // Speed PID: calculate base speed
    float baseSpeed = calcSpeedPID();
    // Tracking PID: calculate correction value
    int trackCorrection = calcTrackPID();

    // Final left/right wheel speed = Speed PID output ± Tracking PID correction
    int leftSpeed = baseSpeed - trackCorrection;
    int rightSpeed = baseSpeed + trackCorrection + RIGHT_COMPENSATION;

    // Force search when line is lost
    if (isLineLost) {
      leftSpeed = (searchDir == 1) ? MAX_MOTOR_SPEED : -MAX_MOTOR_SPEED;
      rightSpeed = (searchDir == 1) ? -MAX_MOTOR_SPEED : MAX_MOTOR_SPEED;
      // Stop search when line is found and minimum search time elapsed
      if ((searchDir == 1 && sensorNorm[7] > TRACK_BLACK_THRESH) || 
          (searchDir == -1 && sensorNorm[0] > TRACK_BLACK_THRESH)) {
        if (now - searchStartTime >= MIN_SEARCH_TIME) isLineLost = false;
      }
    }

    setMotorsLR(leftSpeed, rightSpeed); // Output motor speeds
  }

  // 4. Serial debugging (every 500ms)
  if (now - lastSerialPrint >= 500) {
    lastSerialPrint = now;
    Serial.print("\nState: ");
    Serial.print(currentSpeedState == CONSTANT ? "CONSTANT" : (currentSpeedState == ACCEL ? "ACCEL" : "DECEL"));
    Serial.print(" | SpeedTarget: "); Serial.print(currentSpeedTarget);
    Serial.print(" | TrackError: "); Serial.println(trackError);
  }
}