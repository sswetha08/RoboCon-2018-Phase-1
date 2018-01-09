#include <openGLCD.h>
#define button
void setup() {
  pinMode(button, INPUT);
  // put your setup code here, to run once:
  GLCD.Init();
  GLCD.SelectFont(System5x7);

  GLCD.println("Start test bench....");
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  GLCD.CursorTo(0, 1);
  int buttonstate = digitalRead(button);
  if (buttonstate == HIGH) {
    Serial.write("#bm");
    // tO READ ENCODER VALUES
    for (int i = 0; i < 4; i++) {
      if (Serial.available())
      {
        byte c[2];
        Serial.readBytes(c, 2);
        value[i] = (c[0] << 8) | c[1];
        GLCD.print(i);
        GLCD.println("Encoder value :");
        GLCD.write(value[i]);
      }
    }
    //to read imu values
    byte y = Serial.read();
    GLCD.println(y);
    Serial.write("#l");
  }
}
