#include <openGLCD.h>
#define button
void setup() {
  pinMode(button, INPUT);
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
  int buttonstate = digitalRead(button);
  if (buttonstate == HIGH) {
    Serial2.write("#bm");
    // tO READ ENCODER VALUES
    for (int i = 0; i < 4; i++) {
      if (Serial2.available())
      {
        byte c[2];
        Serial2.readBytes(c, 2);
        value[i] = (c[0] << 8) | c[1];
        GLCD.print(i);
        GLCD.println("Encoder value :");
        GLCD.write(value[i]);
      }
    }
    //to read imu values
    byte y = Serial2.read();
    GLCD.println(y);
    Serial2.write("#l");
  }
}
