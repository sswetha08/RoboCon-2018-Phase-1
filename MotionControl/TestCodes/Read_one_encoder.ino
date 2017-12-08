
#include <Wire.h>

void setup() 
{
  Wire.begin();        
  Serial.begin(9600);  
  Serial3.begin(9600);
}
char control;
long int value[4];
void loop() 
{ 
  if(Serial3.available())
  control = Serial3.read();
  Serial.print(control);
if(control == 'S')
{
 for(int i=1;i<4;i++)
 {
 Wire.beginTransmission(i);
 Wire.requestFrom(i, 4);    
  if(Wire.available()) 
  { 
   byte b4 = Wire.read(); 
   byte b3= Wire.read();
   byte b2= Wire.read();
   byte b1= Wire.read();
   value[i] = (b4<<24) | (b3<<16) | (b2<<8) | b1;    
   Serial.print("Encoder");Serial.print(i-1);Serial.print(" :");Serial.println(value[i]);
   
  }
  Wire.endTransmission();
 }  
}
delay(100);
}

