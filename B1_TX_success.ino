#include <LiquidCrystal.h>
#include <SPI.h>

// ---------------- LCD Pins ----------------
const uint8_t LCD_RS = 4;
const uint8_t LCD_E  = 3;
const uint8_t LCD_D4 = 5;
const uint8_t LCD_D5 = 6;
const uint8_t LCD_D6 = 8;
const uint8_t LCD_D7 = 2;
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ---------------- Joystick Pins ----------------
const uint8_t JOY_X = A0;
const uint8_t JOY_Y = A1;
const uint8_t JOY_SW = 7;

// ---------------- nRF24 Pins ----------------
const uint8_t CE_PIN = 9;
const uint8_t CSN_PIN = 10;

// Joystick parameters
const int JOY_CENTER = 512;
const int DEADZONE = 40;
const bool INVERT_X = false;
const bool INVERT_Y = true;

// nRF24 registers
#define CONFIG      0x00
#define STATUS      0x07
#define FIFO_STATUS 0x17

// Status variables
bool armed = false;
uint32_t packetCount = 0;
uint32_t successCount = 0;
unsigned long lastSend = 0;
unsigned long lastLcd = 0;
unsigned long lastDebug = 0;

// EMA filtering
float emaX = JOY_CENTER, emaY = JOY_CENTER;
const float JOY_ALPHA = 0.2f;

void lcdPrint16(int col, int row, const String& s) {
  lcd.setCursor(col, row);
  lcd.print(s);
  for (int i = col + s.length(); i < 16; ++i) lcd.print(' ');
}

byte readRegister(byte reg) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(reg & 0x1F);
  byte value = SPI.transfer(0x00);
  digitalWrite(CSN_PIN, HIGH);
  return value;
}

void writeRegister(byte reg, byte value) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0x20 | reg);
  SPI.transfer(value);
  digitalWrite(CSN_PIN, HIGH);
}

void setupNrf24() {
  Serial.println("Initializing nRF24...");
  
  // Configure nRF24 as transmitter
  writeRegister(CONFIG, 0x0E);       // PWR_UP=1, PRIM_RX=0 (Transmitter mode)
  writeRegister(0x01, 0x00);         // Disable auto-acknowledgment
  writeRegister(0x02, 0x00);         // Disable RX address
  writeRegister(0x03, 0x03);         // 5-byte address width
  writeRegister(0x04, 0x00);         // Disable retransmission
  writeRegister(0x05, 76);           // Channel 76
  writeRegister(0x06, 0x06);         // 1Mbps data rate, 0dBm output power
  
  // Set TX address
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0x20 | 0x10); // TX_ADDR command
  SPI.transfer('C');
  SPI.transfer('T');
  SPI.transfer('R');
  SPI.transfer('2');
  SPI.transfer('0');
  digitalWrite(CSN_PIN, HIGH);
  
  Serial.println("nRF24 configured as transmitter");
}

// Reset nRF24 status
void resetNrf24() {
  digitalWrite(CE_PIN, LOW);
  delay(10);
  
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0xE1); // FLUSH_TX command
  digitalWrite(CSN_PIN, HIGH);
  
  writeRegister(STATUS, 0x70); // Clear all status flags
}

bool sendPacket(int16_t L, int16_t R, uint8_t buttons) {
  // Ensure data is within valid range
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  
  // Clear status register
  writeRegister(STATUS, 0x70);
  
  // Write data - use correct little-endian format
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0xA0); // W_TX_PAYLOAD command
  
  // Send 5-byte packet: L(2) + R(2) + buttons(1)
  // Little-endian: low byte first, then high byte
  SPI.transfer(L & 0xFF);        // L low byte
  SPI.transfer((L >> 8) & 0xFF); // L high byte
  SPI.transfer(R & 0xFF);        // R low byte
  SPI.transfer((R >> 8) & 0xFF); // R high byte
  SPI.transfer(buttons);         // Button status
  
  digitalWrite(CSN_PIN, HIGH);
  
  // Start transmission
  digitalWrite(CE_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(CE_PIN, LOW);
  
  // Wait for transmission completion
  delay(1);
  
  // Check transmission status
  byte status = readRegister(STATUS);
  bool success = (status & 0x20); // TX_DS flag (Transmit Done)
  
  if (success) {
    writeRegister(STATUS, 0x20); // Clear TX_DS flag
  } else if (status & 0x10) {    // MAX_RT flag (Maximum Retransmit)
    writeRegister(STATUS, 0x10); // Clear MAX_RT flag
    resetNrf24(); // Reset on transmission failure
  }
  
  // Debug output
  if (millis() - lastDebug > 1000) {
    lastDebug = millis();
    Serial.print("TX: L=");
    Serial.print(L);
    Serial.print(" R=");
    Serial.print(R);
    Serial.print(" -> Bytes: 0x");
    Serial.print(L & 0xFF, HEX);
    Serial.print(" 0x");
    Serial.print((L >> 8) & 0xFF, HEX);
    Serial.print(" 0x");
    Serial.print(R & 0xFF, HEX);
    Serial.print(" 0x");
    Serial.print((R >> 8) & 0xFF, HEX);
    Serial.print(" 0x");
    Serial.print(buttons, HEX);
    Serial.print(" | Status: 0x");
    Serial.print(status, HEX);
    Serial.print(" | ");
    Serial.println(success ? "SUCCESS" : "FAILED");
  }
  
  return success;
}

int mapAxisTo255(int adc, bool invert) {
  int v = adc - JOY_CENTER;
  if (invert) v = -v;
  if (abs(v) < DEADZONE) return 0;
  
  int span = 512 - DEADZONE;
  long m = (long)255 * v / span;
  if (m > 255) m = 255;
  if (m < -255) m = -255;
  return (int)m;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Remote Control - FIXED VERSION ===");
  
  // Initialize pins
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);
  digitalWrite(CSN_PIN, HIGH);
  
  // LCD initialization
  lcd.begin(16, 2);
  lcd.clear();
  lcdPrint16(0, 0, "REMOTE CONTROL");
  lcdPrint16(0, 1, "INITIALIZING...");
  
  // Initialize SPI and nRF24
  SPI.begin();
  delay(100);
  setupNrf24();
  
  delay(1000);
  lcd.clear();
  lcdPrint16(0, 0, "READY!");
  lcdPrint16(0, 1, "BTN: ARM/DISARM");
  delay(1000);
  lcd.clear();
  
  Serial.println("Remote control ready - Starting transmission...");
}

void loop() {
  // Read joystick data
  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);
  emaX = (1.0f - JOY_ALPHA) * emaX + JOY_ALPHA * (float)rawX;
  emaY = (1.0f - JOY_ALPHA) * emaY + JOY_ALPHA * (float)rawY;
  
  int x255 = mapAxisTo255((int)emaX, INVERT_X);
  int y255 = mapAxisTo255((int)emaY, INVERT_Y);
  
  // Button detection (safety lock)
  static bool lastBtn = HIGH;
  bool btnNow = digitalRead(JOY_SW); 
  static unsigned long lastDeb = 0;
  if (btnNow != lastBtn && (millis() - lastDeb) > 30) {
    lastDeb = millis();
    lastBtn = btnNow;
    if (btnNow == LOW) {
      armed = !armed;
      Serial.print("Button: ");
      Serial.println(armed ? "ARMED" : "DISARMED");
    }
  }
  
  // Calculate motor speeds (differential steering)
  long L = (long)y255 - (long)x255;
  long R = (long)y255 + (long)x255;
  
  // Limit to valid range
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);
  
  // Send control data (20Hz = every 50ms)
  unsigned long now = millis();
  if (now - lastSend >= 50) {
    lastSend = now;
    
    int16_t sendL = armed ? (int16_t)L : 0;
    int16_t sendR = armed ? (int16_t)R : 0;
    uint8_t buttons = (digitalRead(JOY_SW) == LOW ? 0x01 : 0x00);
    
    bool success = sendPacket(sendL, sendR, buttons);
    if (success) {
      successCount++;
    }
    packetCount++;
    
    // Auto-reset nRF24 every 30 seconds
    static unsigned long lastReset = 0;
    if (now - lastReset > 30000) {
      lastReset = now;
      resetNrf24();
      Serial.println("Auto-reset nRF24");
    }
  }
  
  // Update LCD display (10Hz)
  if (now - lastLcd >= 100) {
    lastLcd = now;
    
    // Line 1: Joystick position and status
    char line0[17];
    snprintf(line0, sizeof(line0), "X:%+4d Y:%+4d", x255, y255);
    lcdPrint16(0, 0, line0);
    
    // Line 2: Motor speed and transmission status
    char line1[17];
    if (armed) {
      snprintf(line1, sizeof(line1), "ARM L:%+4d R:%+4d", 
               (int16_t)L, (int16_t)R);
    } else {
      snprintf(line1, sizeof(line1), "STOP L:   0 R:   0");
    }
    lcdPrint16(0, 1, line1);
    
    // Display transmission status in corner
    lcd.setCursor(15, 0);
    float successRate = (packetCount > 0) ? (successCount * 100.0 / packetCount) : 0;
    if (successRate > 90) {
      lcd.print("+");
    } else if (successRate > 70) {
      lcd.print("-");
    } else {
      lcd.print("!");
    }
  }
  
  delay(2);
}