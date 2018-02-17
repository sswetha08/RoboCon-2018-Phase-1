/*motor direction HIGH for UA launch
   motor reset speed 50

*/

#define M_PWM 9
#define M_DIR 8
#include <Servo.h>
Servo flapser;
#define dep 10
#define ret 100
#define pes_pin 2
#define BTSerial Serial1

void reset_arm()
{
  analogWrite(M_PWM, 0);
  delay(2000);
  digitalWrite(M_DIR, LOW);
  analogWrite(M_PWM, 30);
  while (!digitalRead(pes_pin))
  {}
  delay(1000);
  flapser.write(dep);
  analogWrite(M_PWM, 25);
  delay(980);
  analogWrite(M_PWM, 0);

}


void setup() {
  BTSerial.begin(38400);
  flapser.attach(10);
  pinMode(M_PWM, OUTPUT);
  pinMode(M_DIR, OUTPUT);
  analogWrite(M_PWM, 20);
  digitalWrite(M_DIR, LOW);
  flapser.write(ret);
  reset_arm();
}

void loop()
{
  
  BTSerial.write('x');
  
  boolean detectedShuttlecock = false;
  while (!detectedShuttlecock) {
    for (int i = 0 ; i <= 1900 ; i++) {
      delay(1);
      int LEVEL = digitalRead(pes_pin);
      if (LEVEL == HIGH && i == 1900) {
        detectedShuttlecock = true;
        Serial.println("Detected shuttlecock");
      }
      if (LEVEL == LOW) {
        break;
      }
    }
  }
  BTSerial.write('y');
}

