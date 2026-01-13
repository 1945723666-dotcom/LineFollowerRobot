// ===== Friction Test: Constant Speed (per wheel) using Baseboard Encoders over I2C =====
// Your car motor protocol: 'b' 'a' + dir bytes + uint16 speeds (order 0,2,1,3)
// Encoder read protocol (from other group's working code): 'i' '0' and 'i' '5'
// Serial Plotter output (numbers only): rpm1<TAB>rpm2<TAB>rpm3<TAB>rpm4

#include <Wire.h>
#include <math.h>   // isfinite(), NAN

// ---------------- I2C ----------------
static const uint8_t BOARD_ADDR = 0x2A;   // 42 decimal

// ---------------- Encoder scaling ----------------
// Tune so that displayed RPM matches reality.
// If your plotter shows ~2x too high -> increase TICKS_PER_REV.
// If too low -> decrease.
static const float TICKS_PER_REV = 840.0f;   // <-- start here, then tune

// ---------------- Control loop ----------------
static const uint16_t CONTROL_MS = 50;       // 20 Hz
static const float DT_SEC = (float)CONTROL_MS / 1000.0f;

static const int MIN_CMD = 0;
static const int MAX_CMD = 100;

static float Kp = 0.5f;     // PI for friction compensation
static float Ki = 2.4f;

static float targetRPM = 30.0f;   // desired constant RPM per wheel
static int   baseCmd   = 25;      // feedforward command (0..100)

static const float I_CLAMP = 200.0f;

// Glitch filter: if delta ticks per 50ms exceeds this, ignore sample
static const uint32_t MAX_DELTA_TICKS = 50000UL;

// ---------------- State ----------------
static bool runEnabled = true;  // set false if you want to require 'r' to start

static uint32_t e[4]     = {0,0,0,0};
static uint32_t lastE[4] = {0,0,0,0};
static bool encoderInit = false;

static float rpm[4]      = {0,0,0,0};
static float lastGood[4] = {0,0,0,0};

static float integ[4] = {0,0,0,0};
static int   cmd[4]   = {0,0,0,0};

static unsigned long lastControlMs = 0;
static int i2cFailStreak = 0;
static const int I2C_FAIL_STOP_TH = 10;

// If encoder index != motor index on your chassis, edit this mapping.
// motorCmd[motorIndex] = cmdFromEncoder[encIndex]
static const uint8_t MOTOR_OF_ENCODER[4] = {0, 1, 2, 3}; // default: enc1->m1, enc2->m2, enc3->m3, enc4->m4

// ---------------- Utility ----------------
static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void printPlotter4(float a, float b, float c, float d) {
  Serial.print(a, 1); Serial.print('\t');
  Serial.print(b, 1); Serial.print('\t');
  Serial.print(c, 1); Serial.print('\t');
  Serial.println(d, 1);
}

// ---------------- Motor commands (YOUR CAR PROTOCOL) ----------------
static inline void writeU16LE(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}

// Your car expects: 'b','a' then dir[0],dir[2],dir[1],dir[3] then speed16[0],speed16[2],speed16[1],speed16[3]
void sendMotorSpeeds_m1m2m3m4(int m1, int m2, int m3, int m4) {
  m1 = clampInt(m1, 0, 100);
  m2 = clampInt(m2, 0, 100);
  m3 = clampInt(m3, 0, 100);
  m4 = clampInt(m4, 0, 100);

  // forward only for this test
  const uint16_t s1 = (uint16_t)m1;
  const uint16_t s2 = (uint16_t)m2;
  const uint16_t s3 = (uint16_t)m3;
  const uint16_t s4 = (uint16_t)m4;

  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('b');
  Wire.write('a');

  // directions in your board order: 0,2,1,3
  Wire.write('f'); Wire.write('f'); Wire.write('f'); Wire.write('f');

  // speeds in same order: 0,2,1,3
  writeU16LE(s1);
  writeU16LE(s3);
  writeU16LE(s2);
  writeU16LE(s4);

  Wire.endTransmission();
}

void stopAll() {
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('h');
  Wire.write('a');
  uint8_t st = Wire.endTransmission();

  // Fallback for other firmware variants (harmless if unsupported)
  if (st != 0) {
    Wire.beginTransmission(BOARD_ADDR);
    Wire.write('H');
    Wire.write('a');
    Wire.endTransmission();
  }
}

// ---------------- Encoder reads (I2C baseboard) ----------------
static bool readU32FromWire(uint32_t &out) {
  if (Wire.available() < 4) return false;
  uint32_t v = 0;
  v |= (uint32_t)Wire.read();
  v |= (uint32_t)Wire.read() << 8;
  v |= (uint32_t)Wire.read() << 16;
  v |= (uint32_t)Wire.read() << 24;
  out = v;
  return true;
}

static bool readTwoEnc_tryCmd(char cmdChar, char idxChar, uint32_t &a, uint32_t &b) {
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write((uint8_t)cmdChar);     // 'i' or 'I'
  Wire.write((uint8_t)idxChar);     // '0' or '5'
  if (Wire.endTransmission() != 0) return false;

  delayMicroseconds(700);

  uint8_t n = Wire.requestFrom((int)BOARD_ADDR, 8);
  if (n != 8) {
    while (Wire.available()) Wire.read();
    return false;
  }

  if (!readU32FromWire(a)) return false;
  if (!readU32FromWire(b)) return false;
  return true;
}

static bool readTwoEnc(char idxChar, uint32_t &a, uint32_t &b) {
  // Try lowercase 'i' first, then uppercase 'I' as fallback
  if (readTwoEnc_tryCmd('i', idxChar, a, b)) return true;
  return readTwoEnc_tryCmd('I', idxChar, a, b);
}

static bool readEncodersFromBaseboard(uint32_t &o1, uint32_t &o2, uint32_t &o3, uint32_t &o4) {
  uint32_t a=0,b=0,c=0,d=0;
  if (!readTwoEnc('0', a, b)) return false;   // encoder1, encoder2
  if (!readTwoEnc('5', c, d)) return false;   // encoder3, encoder4
  o1=a; o2=b; o3=c; o4=d;
  return true;
}

// ---------------- Speed compute ----------------
static float safeTicksToRPM(uint32_t cur, uint32_t prev) {
  int32_t d = (int32_t)(cur - prev); // wrap-safe
  if (d < 0) d = -d;

  uint32_t du = (uint32_t)d;
  if (du > MAX_DELTA_TICKS) return NAN;
  if (TICKS_PER_REV <= 1.0f) return NAN;

  float rpm = ((float)du / TICKS_PER_REV) * (60.0f / DT_SEC);
  if (!isfinite(rpm)) return NAN;

  if (rpm < 0) rpm = 0;
  if (rpm > 300.0f) rpm = 300.0f;
  return rpm;
}

// Anti-windup PI (per wheel)
static int piWheel(float target, float measured, float &integState) {
  float err = target - measured;

  integState += err * DT_SEC;
  integState = clampFloat(integState, -I_CLAMP, I_CLAMP);

  float u_unsat = (float)baseCmd + Kp * err + Ki * integState;
  float u = clampFloat(u_unsat, (float)MIN_CMD, (float)MAX_CMD);

  // anti-windup
  if (u != u_unsat) {
    if ((u >= MAX_CMD && err > 0) || (u <= MIN_CMD && err < 0)) {
      integState -= err * DT_SEC;
    }
  }

  return (int)(u + 0.5f);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000); // stable

  stopAll();
  delay(200);

  cmd[0] = cmd[1] = cmd[2] = cmd[3] = clampInt(baseCmd, 0, 100);
  lastControlMs = millis();
}

void loop() {
  // Serial controls (NO TEXT output; keeps plotter clean)
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 's') { runEnabled = false; stopAll(); }
    if (c == 'r') { runEnabled = true; for (int i=0;i<4;i++) integ[i]=0; encoderInit=false; }
    if (c == '+') { targetRPM += 5.0f; }
    if (c == '-') { targetRPM -= 5.0f; if (targetRPM < 0) targetRPM = 0; }
  }

  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_MS) return;
  lastControlMs += CONTROL_MS;

  // Read encoders
  uint32_t r1, r2, r3, r4;
  bool ok = readEncodersFromBaseboard(r1, r2, r3, r4);

  if (!ok) {
    i2cFailStreak++;
    if (i2cFailStreak >= I2C_FAIL_STOP_TH) {
      runEnabled = false;
      stopAll();
    }
    printPlotter4(lastGood[0], lastGood[1], lastGood[2], lastGood[3]);
    return;
  }
  i2cFailStreak = 0;

  e[0]=r1; e[1]=r2; e[2]=r3; e[3]=r4;

  if (!encoderInit) {
    for (int i=0;i<4;i++) lastE[i]=e[i];
    encoderInit = true;
    printPlotter4(0,0,0,0);
    return;
  }

  // Compute rpm with glitch protection
  for (int i=0;i<4;i++) {
    float rRPM = safeTicksToRPM(e[i], lastE[i]);
    lastE[i] = e[i];

    if (isfinite(rRPM)) { rpm[i] = rRPM; lastGood[i] = rRPM; }
    else rpm[i] = lastGood[i];
  }

  // Control
  if (runEnabled) {
    int cmdFromEnc[4];
    for (int i=0;i<4;i++) cmdFromEnc[i] = piWheel(targetRPM, rpm[i], integ[i]);

    // Map encoder-based command to motor index (in case wiring differs)
    int mcmd[4] = {baseCmd, baseCmd, baseCmd, baseCmd};
    for (int enc=0; enc<4; enc++) {
      uint8_t motor = MOTOR_OF_ENCODER[enc];
      if (motor < 4) mcmd[motor] = cmdFromEnc[enc];
    }

    // Send to your motor board protocol (m1,m2,m3,m4)
    sendMotorSpeeds_m1m2m3m4(mcmd[0], mcmd[1], mcmd[2], mcmd[3]);
  }

  // Plotter output: encoder1..encoder4 RPM
  printPlotter4(rpm[0], rpm[1], rpm[2], rpm[3]);
}