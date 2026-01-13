#include <SoftwareSerial.h>
SoftwareSerial hc06(2,3);  // RX, TX

void setup() {
  Serial.begin(9600);      // USB Serial Monitor
  hc06.begin(9600);        // HC-06 default
  Serial.println("HC-06 bridge ready");
}

void loop() {
  if (hc06.available()) Serial.write(hc06.read());
  if (Serial.available()) hc06.write(Serial.read());
}
