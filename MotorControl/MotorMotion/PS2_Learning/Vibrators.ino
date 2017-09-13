#include <Cytron_PS2Shield.h>

Cytron_PS2Shield PS2(2, 3); //Rx and Tx

void setup() {
  PS2.begin(57600);

  Serial.begin(9600);
}

int motor1Speed = 0, motor2Speed = 0;

void loop() {

  if (Serial.available()) {
    int num = Serial.parseInt();
    switch (num) {
      case 1:
        motor1Speed += 51;
        Serial.println("Increasing motor 1 by 51 : New speed is " + String(motor1Speed));
        break;
      case 2:
        motor2Speed += 51;
        Serial.println("Increasing motor 2 by 51 : New speed is " + String(motor2Speed));
        break;
      default:
        Serial.println("OOPS");
        break;
    }
    Serial.flush();
    if (motor1Speed > 255) motor1Speed = 0;
    if (motor2Speed > 255) motor2Speed = 0;
  }
  PS2.vibrate(PS2_MOTOR_1, motor1Speed);
  PS2.vibrate(PS2_MOTOR_2, motor2Speed);
}

