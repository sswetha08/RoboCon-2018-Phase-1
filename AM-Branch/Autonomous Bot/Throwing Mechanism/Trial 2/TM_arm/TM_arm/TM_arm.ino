#include<SoftwareSerial.h>

SoftwareSerial btModule(11,10); // Rx, Tx

void setup() {
  // put your setup code here, to run once:
  btModule.begin(38400);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (btModule.available()) {
    char c = btModule.read();
    Serial.print(c);
  }
}
