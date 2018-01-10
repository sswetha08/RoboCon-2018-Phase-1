#include <openGLCD.h>
#define button1
#define button2
 int buttonstate1;
 byte c[2];
 byte y;
 int buttonstate2;
void setup() {
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  // put your setup code here, to run once:
  GLCD.Init();
  GLCD.SelectFont(System5x7);

  GLCD.println("Start test bench....");
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  GLCD.CursorTo(0, 1);
  buttonstate1 = digitalRead(button1);
  if (buttonstate1 == HIGH) {
    Serial2.write("#bm");
    // tO READ ENCODER VALUES
    for (int i = 0; i < 4; i++) {
      if (Serial2.available())
      {
        Serial2.readBytes(c, 2);
        value[i] = (c[0] << 8) | c[1];
        GLCD.print(i);
        GLCD.println("Encoder value :");
        GLCD.write(value[i]);
      }
    }
    //to read imu values
    y = Serial2.read();
    GLCD.println(y);
  }
    buttonstate2 = digitalRead(button2);
    if(buttonstate2==HIGH){
    Serial2.write("#l");
    }
  }
}
