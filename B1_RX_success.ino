#include <SPI.h>
#include <Wire.h>

// nRF24 Pins
const uint8_t CE_PIN = 9;
const uint8_t CSN_PIN = 10;

// Motor driver board I2C address
const uint8_t BOARD_ADDR = 0x2A; // Hexadecimal value of 42

// nRF24 Registers
#define CONFIG      0x00
#define STATUS      0x07
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define RX_PW_P0    0x11
#define FIFO_STATUS 0x17

// Data packet structure
struct Packet {
  int16_t L;          // Left motor speed
  int16_t R;          // Right motor speed
  uint8_t buttons;    // Button status
};

unsigned long lastRecvTime = 0;  // Timestamp of last received packet
uint32_t packetCount = 0;        // Total received packet counter

void writeU16(uint16_t v) {
  Wire.write((uint8_t)(v & 0xFF));
  Wire.write((uint8_t)((v >> 8) & 0xFF));
}

void setMotorsLR(int valL, int valR) {
  valL = constrain(valL, -255, 255);
  valR = constrain(valR, -255, 255);
  
  char dirL = (valL >= 0) ? 'f' : 'r'; // 'f' = forward, 'r' = reverse
  char dirR = (valR >= 0) ? 'f' : 'r';
  uint16_t spL = map(abs(valL), 0, 255, 0, 100); // Map 0-255 to 0-100
  uint16_t spR = map(abs(valR), 0, 255, 0, 100);

  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('b');
  Wire.write('a');
  Wire.write(dirL); // M1 direction (Left motor)
  Wire.write(dirL); // M3 direction (Left motor)
  Wire.write(dirR); // M2 direction (Right motor)
  Wire.write(dirR); // M4 direction (Right motor)
  writeU16(spL);   // M1 speed value
  writeU16(spL);   // M3 speed value
  writeU16(spR);   // M2 speed value
  writeU16(spR);   // M4 speed value
  Wire.endTransmission();
}

void stopAll() {
  Wire.beginTransmission(BOARD_ADDR);
  Wire.write('h'); // Stop command byte 1
  Wire.write('a'); // Stop command byte 2
  Wire.endTransmission();
}

byte readRegister(byte reg) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(reg & 0x1F);       // Send register read command
  byte value = SPI.transfer(0x00); // Read register value
  digitalWrite(CSN_PIN, HIGH);
  return value;
}

void writeRegister(byte reg, byte value) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0x20 | reg);       // Send register write command
  SPI.transfer(value);            // Write register value
  digitalWrite(CSN_PIN, HIGH);
}

void setRxAddress(const char* addr) {
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0x20 | 0x0A); // RX_ADDR_P0 command (Set RX address for pipe 0)
  for(int i = 0; i < 5; i++) {
    SPI.transfer(addr[i]);   // Write 5-byte address
  }
  digitalWrite(CSN_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Car Receiver - FIXED CONFIG ===");
  
  // Initialize I2C communication
  Wire.begin();
  stopAll(); // Stop motors on startup
  
  // Initialize nRF24 pins
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CE_PIN, LOW);
  digitalWrite(CSN_PIN, HIGH);
  
  // Initialize SPI communication
  SPI.begin();
  delay(100);
  
  Serial.println("Step 1: Resetting nRF24...");
  
  // Reset nRF24 module
  digitalWrite(CE_PIN, LOW);
  delay(10);
  
  // Flush FIFO buffers
  digitalWrite(CSN_PIN, LOW);
  SPI.transfer(0xE1); // FLUSH_TX command (Clear transmit FIFO)
  SPI.transfer(0xE2); // FLUSH_RX command (Clear receive FIFO)
  digitalWrite(CSN_PIN, HIGH);
  
  // Clear status register flags
  writeRegister(STATUS, 0x70);
  
  Serial.println("Step 2: Configuring registers...");
  
  // Configure nRF24 as receiver
  writeRegister(CONFIG, 0x0F);       // PWR_UP=1, PRIM_RX=1 (Receiver mode)
  delay(10);
  writeRegister(EN_AA, 0x00);        // Disable auto-acknowledgment
  writeRegister(EN_RXADDR, 0x01);    // Enable RX address pipe 0
  writeRegister(SETUP_AW, 0x03);     // 5-byte address width
  writeRegister(SETUP_RETR, 0x00);   // Disable retransmission
  writeRegister(RF_CH, 76);          // RF channel 76
  writeRegister(RF_SETUP, 0x06);     // 1Mbps data rate, 0dBm output power
  writeRegister(RX_PW_P0, 5);        // 5-byte payload size for pipe 0
  
  Serial.println("Step 3: Setting RX address...");
  
  // Set RX address
  setRxAddress("CTR20");
  
  // Verify configuration
  Serial.println("Step 4: Verifying configuration...");
  
  byte configReg = readRegister(CONFIG);
  byte statusReg = readRegister(STATUS);
  byte channelReg = readRegister(RF_CH);
  
  Serial.print("CONFIG: 0x"); Serial.println(configReg, HEX);
  Serial.print("STATUS: 0x"); Serial.println(statusReg, HEX);
  Serial.print("RF_CH: "); Serial.println(channelReg);
  
  if (configReg == 0x0F) {
    Serial.println("✅ nRF24 configured as receiver");
  } else {
    Serial.println("❌ Configuration failed!");
    return;
  }
  
  // Start listening for packets
  Serial.println("Step 5: Starting listening...");
  digitalWrite(CE_PIN, HIGH); // Enable receiver mode
  delay(10);
  
  // Final status check
  byte finalStatus = readRegister(STATUS);
  Serial.print("Final STATUS: 0x");
  Serial.println(finalStatus, HEX);
  
  Serial.println("✅ Receiver READY - Listening for data...");
}

bool readPacket(Packet* pkt) {
  byte status = readRegister(STATUS);
  
  if (status & 0x40) { // RX_DR flag (Data Ready - packet received)
    // Read payload data
    digitalWrite(CSN_PIN, LOW);
    SPI.transfer(0x61); // R_RX_PAYLOAD command (Read receive payload)
    
    uint8_t data[5];
    for(int i = 0; i < 5; i++) {
      data[i] = SPI.transfer(0x00); // Read 5 bytes of payload
    }
    digitalWrite(CSN_PIN, HIGH);
    
    // Parse data (little-endian format)
    pkt->L = (int16_t)((data[1] << 8) | data[0]);  // Left speed (2 bytes)
    pkt->R = (int16_t)((data[3] << 8) | data[2]);  // Right speed (2 bytes)
    pkt->buttons = data[4];                        // Button status (1 byte)
    
    // Clear RX_DR flag
    writeRegister(STATUS, 0x40);
    
    return true;
  }
  
  return false;
}

void loop() {
  Packet pkt;
  
  if (readPacket(&pkt)) {
    packetCount++;
    lastRecvTime = millis(); // Update last received timestamp
    
    Serial.print("✅ Packet ");
    Serial.print(packetCount);
    Serial.print(": L=");
    Serial.print(pkt.L);
    Serial.print(" R=");
    Serial.print(pkt.R);
    Serial.print(" BTN=0x");
    Serial.println(pkt.buttons, HEX);
    
    // Control motors with received values
    setMotorsLR(pkt.L, pkt.R);
    
    // Visual feedback - blink built-in LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Status monitoring (every 3 seconds)
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 3000) {
    lastStatus = millis();
    
    byte status = readRegister(STATUS);
    byte fifo = readRegister(FIFO_STATUS);
    
    Serial.print("📡 Status:0x");
    Serial.print(status, HEX);
    Serial.print(" FIFO:0x");
    Serial.print(fifo, HEX);
    Serial.print(" PKTS:");
    Serial.println(packetCount);
  }
  
  // Timeout protection - stop motors if no packet received for 200ms
  if (millis() - lastRecvTime > 200) {
    stopAll();
  }
  
  delay(1);
}