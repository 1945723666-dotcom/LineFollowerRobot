// ===== B3 Bluetooth Control - Instant Stop Version =====
#include <Wire.h>
#include <SoftwareSerial.h>

// ---------- HC-06 Bluetooth ----------
const uint8_t BT_RX = 2; // Nano receives on D10 (HC-06 TXD -> D10)
const uint8_t BT_TX = 3; // Nano transmits on D11 (D11 -> HC-06 RXD)
SoftwareSerial BT(BT_RX, BT_TX);

// ---------- Baseboard I2C ----------
const uint8_t BOARD_ADDR = 0x2A; // Hexadecimal value of 42

// ---------- Timing / safety ----------
const unsigned long WATCHDOG_MS = 1000;  // Watchdog timeout: 1 second
unsigned long lastCmdMs = 0;

// ---------- Slew limiting (smooth starts) ----------
int16_t tgtL = 0, tgtR = 0;   // Target speeds from phone app
int16_t curL = 0, curR = 0;   // Actual speeds sent to motors
const int MAX_STEP = 20;      // Acceleration step value

// ---------- Speed parameters ----------
const int MAX_SPEED = 255;    // Maximum speed
const int HIGH_SPEED = 200;   // High speed
const int MED_SPEED = 150;    // Medium speed
const int LOW_SPEED = 100;    // Low speed

// ---------- Helpers ----------
void writeU16(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}

void base_stopAll() {
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('h'); Wire.write('a'); // "ha" command for stop
  Wire.endTransmission();
  Serial.println("Motors STOPPED");
}

// Use correct motor control protocol
void base_setLR(int valL, int valR) {
  valL = constrain(valL, -MAX_SPEED, MAX_SPEED);
  valR = constrain(valR, -MAX_SPEED, MAX_SPEED);
  
  char dirL = (valL >= 0) ? 'f' : 'r'; // 'f' = forward, 'r' = reverse
  char dirR = (valR >= 0) ? 'f' : 'r';
  uint16_t spL = map(abs(valL), 0, MAX_SPEED, 0, 100); // Map 0-255 to 0-100
  uint16_t spR = map(abs(valR), 0, MAX_SPEED, 0, 100);

  Serial.print("Motors: L=");
  Serial.print(valL);
  Serial.print("(");
  Serial.print(dirL);
  Serial.print(",sp=");
  Serial.print(spL);
  Serial.print(") R=");
  Serial.print(valR);
  Serial.print("(");
  Serial.print(dirR);
  Serial.print(",sp=");
  Serial.print(spR);
  Serial.println(")");
  
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('b'); Wire.write('a'); // "ba" command for motor control
  Wire.write(dirL);        // M1 Left motor direction
  Wire.write(dirL);        // M3 Left motor direction  
  Wire.write(dirR);        // M2 Right motor direction
  Wire.write(dirR);        // M4 Right motor direction
  writeU16(spL);          // M1 speed value
  writeU16(spL);          // M3 speed value
  writeU16(spR);          // M2 speed value
  writeU16(spR);          // M4 speed value
  Wire.endTransmission();
}

// ---------- Command parsing ----------
bool parseLineToLR(const char* s, int &L, int &R) {
  const char* c = strchr(s, ',');
  if (!c) return false;
  
  char *endp = nullptr;
  long l = strtol(s, &endp, 10);
  if (endp == s) return false;
  
  long r = strtol(c+1, nullptr, 10);

  L = (int)constrain(l, -MAX_SPEED, MAX_SPEED);
  R = (int)constrain(r, -MAX_SPEED, MAX_SPEED);
  return true;
}

bool readBTLine(char* buf, size_t maxlen) {
  static size_t idx = 0;
  while (BT.available()) {
    char ch = BT.read();
    if (ch == '\r') continue; // Ignore carriage return
    if (ch == '\n') {         // Line feed = end of command
      buf[idx] = '\0';
      idx = 0;
      return true;
    }
    if (idx < maxlen - 1) {   // Store character if buffer has space
      buf[idx++] = ch;
    } else {                  // Buffer overflow, reset
      idx = 0;
    }
  }
  return false;
}

static inline int16_t approach(int16_t cur, int16_t tgt) {
  if (cur < tgt) { 
    cur += MAX_STEP; 
    if (cur > tgt) cur = tgt; 
  } else if (cur > tgt) { 
    cur -= MAX_STEP; 
    if (cur < tgt) cur = tgt; 
  }
  return cur;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  base_stopAll(); // Stop motors on startup

  BT.begin(9600); // Initialize Bluetooth at 9600 baud rate
  
  Serial.println(F("=== Bluetooth Car Control - Instant Stop ==="));
  Serial.println(F("Send: L,R  like '200,200' or '-200,-200'"));
  Serial.println(F("Or: FWD, REV, LEFT, RIGHT, STOP, FAST, SLOW"));
  Serial.println(F("Stop commands will INSTANTLY stop motors"));
  
  BT.println(F("HC-06 ready. Send commands..."));

  lastCmdMs = millis(); // Initialize last command timestamp
}

void loop() {
  char line[32];
  if (readBTLine(line, sizeof(line))) { // Read complete command line
    Serial.print("Received: ");
    Serial.println(line);
    
    int L, R;
    if (parseLineToLR(line, L, R)) { // Parse numeric L,R command
      Serial.print("Parsed: L=");
      Serial.print(L);
      Serial.print(" R=");
      Serial.println(R);
      
      // Instant stop if command is 0,0
      if (L == 0 && R == 0) {
        curL = 0;
        curR = 0;
        base_stopAll();
        Serial.println("INSTANT STOP");
      } else {
        tgtL = L; 
        tgtR = R;
      }
      lastCmdMs = millis(); // Update command timestamp
    } else {
      // Text command parsing
      if (!strcmp(line, "FWD")) { 
        tgtL = MED_SPEED; tgtR = MED_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: FORWARD (150)");
      }
      else if (!strcmp(line, "REV")) { 
        tgtL = -MED_SPEED; tgtR = -MED_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: REVERSE (150)");
      }
      else if (!strcmp(line, "FAST")) { 
        tgtL = HIGH_SPEED; tgtR = HIGH_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: FAST (200)");
      }
      else if (!strcmp(line, "FULL")) { 
        tgtL = MAX_SPEED; tgtR = MAX_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: FULL SPEED (255)");
      }
      else if (!strcmp(line, "SLOW")) { 
        tgtL = LOW_SPEED; tgtR = LOW_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: SLOW (100)");
      }
      else if (!strcmp(line, "LEFT")) { 
        tgtL = -HIGH_SPEED; tgtR = HIGH_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: TURN LEFT");
      }
      else if (!strcmp(line, "RIGHT")) { 
        tgtL = HIGH_SPEED; tgtR = -HIGH_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: TURN RIGHT");
      }
      else if (!strcmp(line, "SPINL")) { 
        tgtL = -MAX_SPEED; tgtR = MAX_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: SPIN LEFT");
      }
      else if (!strcmp(line, "SPINR")) { 
        tgtL = MAX_SPEED; tgtR = -MAX_SPEED; 
        lastCmdMs = millis(); 
        Serial.println("Command: SPIN RIGHT");
      }
      else if (!strcmp(line, "STOP")) { 
        // Execute stop command immediately
        tgtL = 0; tgtR = 0;
        curL = 0; curR = 0;
        base_stopAll();
        lastCmdMs = millis(); 
        Serial.println("Command: INSTANT STOP");
      }
    }
  }

  // Watchdog stop (execute immediately)
  if (millis() - lastCmdMs > WATCHDOG_MS) {
    if (tgtL != 0 || tgtR != 0) {
      tgtL = 0; 
      tgtR = 0;
      curL = 0;
      curR = 0;
      base_stopAll();
      Serial.println("Watchdog: INSTANT STOP");
    }
  }

  // Use smooth control only for non-zero targets
  if (tgtL != 0 || tgtR != 0) {
    curL = approach(curL, tgtL);
    curR = approach(curR, tgtR);
    
    if (curL != 0 || curR != 0) {
      base_setLR(curL, curR);
    }
  }

  delay(20); // Small delay for stability
}