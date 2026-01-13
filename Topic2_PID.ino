#include <Wire.h>
#include <EEPROM.h>

/* ===================== MOTOR BOARD (UNCHANGED) ===================== */
const uint8_t BOARD_ADDR = 0x2A;

static inline void writeU16(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}

void setMotorsLR(int16_t valL, int16_t valR) {
  valL = constrain(valL, -255, 255);
  valR = constrain(valR, -255, 255);

  char dirL = (valL >= 0) ? 'f' : 'r';
  char dirR = (valR >= 0) ? 'f' : 'r';

  uint16_t spL = map(abs(valL), 0, 255, 0, 100);
  uint16_t spR = map(abs(valR), 0, 255, 0, 100);

  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('b'); Wire.write('a');
  Wire.write(dirL); Wire.write(dirL);
  Wire.write(dirR); Wire.write(dirR);
  writeU16(spL); writeU16(spL);
  writeU16(spR); writeU16(spR);
  Wire.endTransmission();
}

void stopAll() {
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('h'); Wire.write('a');
  Wire.endTransmission();
}

/* ===================== SENSOR PINS ===================== */
const uint8_t PIN_L = A1;
const uint8_t PIN_R = A0;

/* ===================== EEPROM CALIB STORAGE ===================== */
struct CalData {
  uint16_t magic;
  int16_t  Lw, Lb, Rw, Rb;
  uint8_t  chk;
};
const uint16_t CAL_MAGIC = 0xBEEF;

uint8_t checksumCal(const CalData &c);
bool    loadCal(CalData &c);
void    saveCal(CalData &c);

uint8_t checksumCal(const CalData &c) {
  uint16_t s = 0;
  s ^= (uint16_t)c.Lw; s ^= (uint16_t)c.Lb; s ^= (uint16_t)c.Rw; s ^= (uint16_t)c.Rb;
  s ^= c.magic;
  return (uint8_t)(s ^ (s >> 8));
}

bool loadCal(CalData &c) {
  EEPROM.get(0, c);
  if (c.magic != CAL_MAGIC) return false;
  if (c.chk != checksumCal(c)) return false;
  if (c.Lb <= c.Lw + 10) return false;
  if (c.Rb <= c.Rw + 10) return false;
  return true;
}

void saveCal(CalData &c) {
  c.magic = CAL_MAGIC;
  c.chk = checksumCal(c);
  EEPROM.put(0, c);
}

/* ===================== CALIB VALUES ===================== */
int L_white = 80, L_black = 550;
int R_white = 80, R_black = 550;

/* ===================== FILTERING ===================== */
const float SENSOR_ALPHA = 0.25f;
float fL = 0, fR = 0;

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}
int readAnalogMedian3(uint8_t pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);
  return median3(a, b, c);
}
// Normalization: 0=pure white, 1=pure black (keep original logic for easy judgment)
float norm2pt(int x, int w, int b) {
  if (b == w) return 0.0f;
  float v = (float)(x - w) / (float)(b - w);
  return constrain(v, 0.0f, 1.0f);
}

/* ===================== CORE PARAMETERS (adapted for "white as reference") ===================== */
int baseSpeed = 45;                    // Straight speed when on full white
const int CORRECT_SPEED_OFFSET = 30;   // Correction amplitude when black line detected (smaller = more stable)
const float BLACK_DETECT_TH = 0.15f;   // Black line detection threshold (>0.15 = black line detected)
const float FULL_WHITE_TH = 0.10f;     // Full white judgment threshold (<0.10 = pure white)

/* ===================== SEAM / LOST LOGIC (retained) ===================== */
const uint16_t GAP_HOLD_MS     = 130;
const uint16_t LOST_ENTER_MS   = 260;
uint32_t lastSeenLineMs = 0;
bool lineLost = false;
uint32_t lostSince = 0;
int lostCount = 0;
const int LOST_COUNT_ON  = 7;
const int LOST_COUNT_OFF = 2;
int16_t lastCmdL = 0, lastCmdR = 0;
int RECOVER_FWD = 65;
int RECOVER_REV = -55;

/* ===================== TIMING ===================== */
uint32_t lastUs = 0;

void printCal() {
  Serial.print(F("L_white=")); Serial.print(L_white);
  Serial.print(F(" L_black=")); Serial.print(L_black);
  Serial.print(F(" R_white=")); Serial.print(R_white);
  Serial.print(F(" R_black=")); Serial.println(R_black);
}

void calibrateWhite() {
  long sL=0, sR=0;
  for (int i=0;i<80;i++){ sL+=analogRead(PIN_L); sR+=analogRead(PIN_R); delay(4); }
  L_white = sL/80; R_white = sR/80;
}
void calibrateBlack() {
  long sL=0, sR=0;
  for (int i=0;i<80;i++){ sL+=analogRead(PIN_L); sR+=analogRead(PIN_R); delay(4); }
  L_black = sL/80; R_black = sR/80;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(150);

  stopAll();
  delay(200);

  // Seed filter
  for (int i=0;i<15;i++){
    fL = analogRead(PIN_L);
    fR = analogRead(PIN_R);
    delay(10);
  }

  CalData c;
  if (loadCal(c)) {
    L_white = c.Lw; L_black = c.Lb; R_white = c.Rw; R_black = c.Rb;
    Serial.println(F("Loaded calibration from EEPROM."));
  } else {
    Serial.println(F("No valid EEPROM calibration. Use w/b then it will be saved."));
  }

  Serial.println(F("\nCommands:"));
  Serial.println(F("  w = calibrate WHITE (and save)"));
  Serial.println(F("  b = calibrate BLACK (and save)"));
  Serial.println(F("  p = print calibration"));
  Serial.println(F("  5/6 baseSpeed +/-   7/8 CORRECT_OFFSET +/-"));
  Serial.println(F("  s = stop"));
  printCal();

  lastUs = micros();
  lastSeenLineMs = millis();
}

void loop() {
  // ---- serial tuning / calibration ----
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c=='w') {
      stopAll(); delay(150);
      calibrateWhite();
      CalData cd; cd.Lw=L_white; cd.Lb=L_black; cd.Rw=R_white; cd.Rb=R_black;
      saveCal(cd);
      Serial.println(F("WHITE calibrated + saved."));
      printCal();
    }
    if (c=='b') {
      stopAll(); delay(150);
      calibrateBlack();
      CalData cd; cd.Lw=L_white; cd.Lb=L_black; cd.Rw=R_white; cd.Rb=R_black;
      saveCal(cd);
      Serial.println(F("BLACK calibrated + saved."));
      printCal();
    }
    if (c=='p') printCal();
    if (c=='s') { stopAll(); lastCmdL=lastCmdR=0; }

    if (c=='5') baseSpeed = min(90, baseSpeed + 5);
    if (c=='6') baseSpeed = max(15, baseSpeed - 5);
    if (c=='7') CORRECT_SPEED_OFFSET = min(50, CORRECT_SPEED_OFFSET + 2); // Increase correction amplitude
    if (c=='8') CORRECT_SPEED_OFFSET = max(10, CORRECT_SPEED_OFFSET - 2); // Decrease correction amplitude
  }

  // ---- dt (delta time) ----
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.050f) dt = 0.050f;

  // ---- read + filter sensors ----
  int rawL = readAnalogMedian3(PIN_L);
  int rawR = readAnalogMedian3(PIN_R);

  fL = SENSOR_ALPHA * rawL + (1.0f - SENSOR_ALPHA) * fL;
  fR = SENSOR_ALPHA * rawR + (1.0f - SENSOR_ALPHA) * fR;

  float nL = norm2pt((int)fL, L_white, L_black);
  float nR = norm2pt((int)fR, R_white, R_black);

  // ---- Core: Line tracking logic (white as reference) ----
  bool isLeftBlack = (nL > BLACK_DETECT_TH);  // Left sensor detects black line
  bool isRightBlack = (nR > BLACK_DETECT_TH); // Right sensor detects black line
  bool isFullWhite = (nL < FULL_WHITE_TH) && (nR < FULL_WHITE_TH); // Full white = go straight

  uint32_t nowMs = millis();
  bool presentNow = (nL > BLACK_DETECT_TH) || (nR > BLACK_DETECT_TH);
  if (presentNow) lastSeenLineMs = nowMs;
  uint32_t missingFor = nowMs - lastSeenLineMs;

  // ---- seam gap-hold ----
  if (!presentNow && missingFor <= GAP_HOLD_MS) {
    setMotorsLR(lastCmdL, lastCmdR);
    return;
  }

  // ---- lost debounce ----
  bool lossCandidate = (!presentNow && missingFor > 0);
  if (lossCandidate) lostCount = min(lostCount + 1, 20);
  else              lostCount = max(lostCount - 1, 0);

  bool shouldBeLost  = (missingFor > LOST_ENTER_MS) && (lostCount >= LOST_COUNT_ON);
  bool shouldRecover = (lostCount <= LOST_COUNT_OFF);

  if (shouldBeLost && !lineLost) {
    lineLost = true;
    lostSince = nowMs;
  } else if (lineLost && presentNow && shouldRecover) {
    lineLost = false;
  }

  // ---- lost recovery (retain original logic) ----
  if (lineLost) {
    uint32_t t = nowMs - lostSince;

    if (t < 120) {
      lastCmdL = baseSpeed;
      lastCmdR = baseSpeed;
      setMotorsLR(lastCmdL, lastCmdR);
      return;
    }

    if (nL > nR) { // Last detected black on left
      lastCmdL = RECOVER_REV;
      lastCmdR = RECOVER_FWD;
    } else {
      lastCmdL = RECOVER_FWD;
      lastCmdR = RECOVER_REV;
    }
    setMotorsLR(lastCmdL, lastCmdR);
    return;
  }

  // ---- Full white straight / Black detected correction ----
  int16_t cmdL = baseSpeed;
  int16_t cmdR = baseSpeed;

  if (isFullWhite) {
    // Full white = no correction, go straight directly (utilize advantage of no error in white area)
    cmdL = baseSpeed;
    cmdR = baseSpeed;
    Serial.println("[Full White] Go straight");
  } else {
    // Black line detected = trigger correction (only small correction to avoid jitter)
    if (isLeftBlack && !isRightBlack) {
      // Left black right white → correct to turn right (left fast, right slow)
      cmdL = baseSpeed + CORRECT_SPEED_OFFSET;
      cmdR = baseSpeed - CORRECT_SPEED_OFFSET;
      Serial.println("[Left Black] Correct to turn right");
    } else if (isRightBlack && !isLeftBlack) {
      // Right black left white → correct to turn left (left slow, right fast)
      cmdL = baseSpeed - CORRECT_SPEED_OFFSET;
      cmdR = baseSpeed + CORRECT_SPEED_OFFSET;
      Serial.println("[Right Black] Correct to turn left");
    } else if (isLeftBlack && isRightBlack) {
      // Both black → go straight with small speed (avoid over correction)
      cmdL = baseSpeed;
      cmdR = baseSpeed;
      Serial.println("[Both Black] Go straight");
    }
  }

  // ---- Execute motor control ----
  cmdL = constrain(cmdL, 0, 255); // Forward only, cancel reverse (simplify logic)
  cmdR = constrain(cmdR, 0, 255);
  setMotorsLR(cmdL, cmdR);
  lastCmdL = cmdL;
  lastCmdR = cmdR;

  // ---- Serial print debugging (optional) ----
  Serial.print("nL:"); Serial.print(nL, 2);
  Serial.print(" | nR:"); Serial.print(nR, 2);
  Serial.print(" | L:"); Serial.print(cmdL);
  Serial.print(" R:"); Serial.println(cmdR);
}