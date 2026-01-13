#include <SPI.h>   //  nRF24 talks over SPI
#include <RF24.h>  //  radio driver.

RF24 rf24(9,10);                 // CE, CSN
const byte addr[] = "1Node";     // 5-byte address

const uint8_t GROUP = 20;        // <-- change to your group
const uint8_t CHANNEL = 129 - 2 * GROUP;
void setup() {
  rf24.begin();
  rf24.setChannel(CHANNEL);      // group-specific channel
  rf24.setPALevel(RF24_PA_MAX);      //TX power 
rf24.setDataRate(RF24_2MBPS);  // 2 Mbps


rf24.openWritingPipe(addr);   // choose destination address
  rf24.stopListening();   // TX mode (transmit-only).
}

void loop() {
  const char msg[] = "Happy Hacking!";  //a small text payload
rf24.write(&msg, sizeof(msg));

  delay(500);  // half-second repeat
}