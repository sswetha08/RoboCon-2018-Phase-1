#define SERVO_PIN 3
#include<Servo.h>
Servo myServo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  Serial.println("Enter angle in degrees");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    int number = Serial.parseInt();
    myServo.write(number);
    Serial.println(number);
  }
}
