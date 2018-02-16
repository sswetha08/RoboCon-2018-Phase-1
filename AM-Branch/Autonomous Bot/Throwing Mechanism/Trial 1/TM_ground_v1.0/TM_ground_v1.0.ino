volatile unsigned long count, temp = 0;
int IR_PIN[8];
int enc_trig = 4075 - 10;
#include <Servo.h>
#define M_PWM 6
#define M_DIR 7
Servo flapser;
#define deploy 90
#define retract 180

void setup() {
  flapser.attach(8);
  flapser.write(retract);
  Serial1.begin(38400);
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, countfn, RISING);
  attachInterrupt(1, dirfn, RISING);
  for (int i = 31, index = 0; i <= 41 ; i += 2, index++) {
    pinMode(i, OUTPUT);
    IR_PIN[index] = i;
  }
  for (int i = 31 ; i <= 37; i += 2) {
    digitalWrite(i, LOW);
  }
  pinMode(M_PWM, OUTPUT);
  pinMode(M_DIR, OUTPUT);
  digitalWrite(M_DIR, HIGH);
}
int PWM, IR;
void loop() {
  if (count != temp)
  {
    Serial.println(count);
    temp = count;
  }
  while (Serial.available()) {
    char c = Serial.read();
    switch (c)
    {
      case 'M':
      case 'm':
        PWM = Serial.parseInt();
        Serial.print(PWM);
        Serial.println(" to PWM");
        digitalWrite(M_DIR, HIGH);
          analogWrite(M_PWM, PWM);
          flapser.write(retract);
        /*while (abs(flapser.read() - 90) <= 4)
        {
          digitalWrite(M_DIR, HIGH);
          analogWrite(M_PWM, PWM);
        }*/
        break;
        case 'a':case 'A':
        enc_trig = Serial.parseInt();
        Serial.print("TRIG to : ");
        Serial.println(enc_trig);
        break;
      case 's':
      case 'S':
     // case '1':

        // flapser.write(retract);
        digitalWrite(39, HIGH);
        digitalWrite(37, HIGH);
        digitalWrite(33, HIGH);
        while (1)
        {
          analogWrite(M_PWM, 20);
          digitalWrite(M_DIR, LOW);
          if (Serial1.available())
          {
            char c = Serial1.read();
            if (c == 'b')
            {
              analogWrite(M_PWM, 0);
              flapser.write(deploy);
              delay(500);
              analogWrite(M_PWM, 20);
              break;
            }
          }
        }
        count = 0;
        Serial.print("The count is : ");
        Serial.println(count);
        while(count >= - 80);
        Serial1.write('x'); // arm opens
        Serial.println("Enter 'y' or '1' : ");
        while(1) {
          if (Serial.available()) {
            char c = Serial.read();
            if (c == 'l' || c == 'y') {
              Serial1.write('y'); // arm close
        digitalWrite(39, LOW);
        digitalWrite(37, LOW);
        digitalWrite(35, LOW);
              break;
            }
          }
        }
        count = 0;
        flapser.write(retract);
        analogWrite(M_PWM, 0);
        digitalWrite(M_DIR, HIGH);
        digitalWrite(39, LOW);
        digitalWrite(37, LOW);
        digitalWrite(35, LOW);
                break;
      case 'I':
      case 'i':
        IR = Serial.parseInt();
        Serial.print(IR);
        Serial.println(" to IR");
        for (int i = 31 ; i <= 37; i++)
        {
          digitalWrite(i, LOW);
        }
        break;
      default: break;
    }
  }
  if (count >= enc_trig)
  {
    Serial1.write('x');
    delay(1000);
    Serial1.write('y');
    digitalWrite(IR_PIN[IR], HIGH);
    delay(1000);
    digitalWrite(IR_PIN[IR], LOW);
    PWM = 0;
    delay(2000);
    digitalWrite(M_PWM, PWM);
    count = 0;
    flapser.write(retract);
    while(Serial1.available()) {
      Serial.print(Serial1.read());
    }
  }

}


void countfn()
{
  if (digitalRead(3) == LOW)
  {
    count--;
  }

}

void dirfn()
{
  if (digitalRead(2) == LOW)
  {
    count++;
  }
}
