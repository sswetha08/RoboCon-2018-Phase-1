// #include<SoftwareSerial.h>

// SoftwareSerial IMU(6,5);  //Rx, Tx

#define IMU Serial1

void setup() {
  Serial.begin(57600);
  IMU.begin(57600);
}

void loop() {
  while (IMU.available()) {
    char c = IMU.read();
    if (c == 'Y') {
      Serial.print("Yaw is : ");
      int angle = IMU.parseInt();
      Serial.println(angle);
    }
  }
}
