
// Master code for i2c communication 
#include <Wire.h>

void setup() 
{
  Wire.begin();        
  Serial.begin(9600);  
}

void loop() 
{ 
  //to send value
 Wire.beginTransmission(0x20);
 byte value=Serial.read();
 Wire.write(value);
 Wire.endTransmission();
//to receive value
 Wire.beginTransmission(0x20);
 delay(100);
 Wire.requestFrom(0x20,1);
  if(Wire.available()) 
  { 
   byte b4 = Wire.read(); 
   Serial.println(b4);
  }
  Wire.endTransmission();
  delay(100);
}
