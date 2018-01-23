#include<Wire.h>

volatile long encoder;
int Channel_A = 2;
int Channel_B = 3;
void setup() {
  Wire.begin(3);
  Serial.begin(9600);
  Wire.onRequest(send_encoder);
  pinMode(Channel_A, INPUT_PULLUP);
  pinMode(Channel_B, INPUT_PULLUP);
  attachInterrupt( digitalPinToInterrupt(Channel_A), counting, RISING);

}

void loop() {
  
 // Serial.println(encoder);
}
void counting() {
  if (digitalRead(Channel_B) == 1) {
    encoder++;
  }
  else {
    encoder--;
  }
}
void send_encoder()
{
  byte b4 = (encoder>>24);
  Wire.write(b4);
  byte b3 = ((encoder>>16) & 0xff);
  Wire.write(b3);
  byte b2 = ((encoder>>8) & 0xff);
  Wire.write(b2);
  byte b1 = (encoder & 0xff);
  Wire.write(b1);
}

