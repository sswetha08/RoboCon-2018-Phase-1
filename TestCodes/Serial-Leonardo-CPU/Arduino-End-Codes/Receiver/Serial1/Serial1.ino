void setup() {
  pinMode(13, OUTPUT);
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
    switch (c) {
      case '0':
      digitalWrite(13, LOW);
      break;
      case '1':
      digitalWrite(13, HIGH);
      break;
      default:
      break;
    }
  }
  if (Serial.available()) {
    Serial1.print(Serial.read());
  }
}
