#include <SPI.h>
#include <RF24.h>

RF24 rf24(9,10);                 // CE, CSN
const byte addr[] = "1Node";
const byte pipe = 1;

const uint8_t GROUP = 20;        // <-- same group
const uint8_t CHANNEL = 129 - 2 * GROUP;

void setup() {
  Serial.begin(9600);  // for printing to the PC.
  rf24.begin();
  rf24.setChannel(CHANNEL);
  rf24.setPALevel(RF24_PA_MAX);
rf24.setDataRate(RF24_2MBPS);


  rf24.openReadingPipe(pipe, addr); // listen on address
  rf24.startListening();  // RX mode
  Serial.println("nRF24L01 ready!");
}

void loop() {
  if (rf24.available()) {
    char msg[32] = {0}; // RX buffer
    rf24.read(&msg, sizeof(msg));
    Serial.println(msg); // show it
  }
}