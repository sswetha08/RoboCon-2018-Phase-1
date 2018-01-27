#include<SoftwareSerial.h>

SoftwareSerial assistController(8,9); // Rx and Tx

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  assistController.begin(9600);

  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())  {
    char c = Serial.read();
    if (c == '1') {
      digitalWrite(13, HIGH);
    } else if (c == '0') {
      digitalWrite(13, LOW);
    }
    assistController.print(c);
  }
  if (assistController.available()) {
    Serial.write(assistController.read());
  }
}
