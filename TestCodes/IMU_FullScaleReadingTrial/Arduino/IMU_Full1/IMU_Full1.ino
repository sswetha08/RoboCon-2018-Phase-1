#include<SoftwareSerial.h>
SoftwareSerial IMU(6,7);  //Rx,Tx of IMU
void setup() {
  // put your setup code here, to run once:
   IMU.begin(57600);
   Serial.begin(57600);
}

int angle;
int fullScaleAngle;
int firstReading; boolean firstReadingDone = true;

void loop() {
  // put your main code here, to run repeatedly:
  if (IMU.available()) {
    char c = IMU.read();
    if (c == 'Y') {
      angle = IMU.parseInt();
      if (!firstReadingDone) {
        firstReading = angle;
        firstReadingDone = true;
      }
    }
  }

  fullScaleAngle = (angle >= 0) ? angle : (360 + angle);
  Serial.print("Actual angle = ");
  Serial.print(angle);
  Serial.print("\tFullscale angle = ");
  Serial.print(fullScaleAngle);
  Serial.print("\tRelAngle = ");
  Serial.print(fullScaleAngle - firstReading);
  Serial.println();
}
